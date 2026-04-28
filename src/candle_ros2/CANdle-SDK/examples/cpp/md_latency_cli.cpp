#include "candle.hpp"
#include "MD.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sched.h>

namespace
{

constexpr double kFrameTimePerTransferUs = 407.0;

enum class BatchMode
{
    Sequential,
    Async,
};

struct Settings
{
    std::vector<int> md_ids;
    mab::candleTypes::busTypes_t bus = mab::candleTypes::busTypes_t::USB;
    std::optional<int> priority;
    BatchMode mode = BatchMode::Sequential;
    int samples = 1000;
    bool show_help = false;
};

struct Summary
{
    double average_batch_us = 0.0;
    double batch_frequency_khz = 0.0;
    double average_per_drive_us = 0.0;
    double estimated_frame_time_us = 0.0;
    double estimated_host_overhead_us = 0.0;
    double estimated_bus_utilization_pct = 0.0;
    size_t failed_samples = 0;
    size_t failed_requests = 0;
};

void print_usage(const char * program)
{
    std::cout
        << "Usage: " << program
        << " --id <can_id> [--id <can_id> ...] [--bus usb|spi] [--priority <0-99>]\n"
        << "       [--mode sequential|async] [--samples <count>]\n"
        << "\n"
        << "Runs the CANdle/MD latency test at 1 Mbit CAN rate using the vendored SDK.\n"
        << "Repeat --id to benchmark multiple drives in one measured batch.\n"
        << "\n"
        << "Examples:\n"
        << "  " << program << " --id 37 --bus spi --priority 99\n"
        << "  " << program
        << " --id 37 --id 940 --id 941 --bus spi --priority 99 --mode async\n";
}

std::optional<int> parse_int(const std::string & value)
{
    try
    {
        size_t consumed = 0;
        const int parsed = std::stoi(value, &consumed, 10);
        if (consumed != value.size())
        {
            return std::nullopt;
        }
        return parsed;
    }
    catch (const std::exception &)
    {
        return std::nullopt;
    }
}

const char * mode_name(const BatchMode mode)
{
    switch (mode)
    {
        case BatchMode::Sequential:
            return "sequential";
        case BatchMode::Async:
            return "async";
        default:
            return "unknown";
    }
}

const char * bus_name(const mab::candleTypes::busTypes_t bus)
{
    switch (bus)
    {
        case mab::candleTypes::busTypes_t::USB:
            return "usb";
        case mab::candleTypes::busTypes_t::SPI:
            return "spi";
        default:
            return "unknown";
    }
}

bool set_fifo_priority(const int priority)
{
    if (priority < 0 || priority > 99)
    {
        std::cerr << "Invalid priority " << priority << ". Expected range 0..99.\n";
        return false;
    }

    sched_param sp{};
    sp.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
    {
        std::cerr << "Failed to set SCHED_FIFO priority " << priority
                  << ": " << std::strerror(errno) << "\n";
        return false;
    }

    return true;
}

bool parse_args(int argc, char ** argv, Settings & settings)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            settings.show_help = true;
            return true;
        }
        if (arg == "--id" && i + 1 < argc)
        {
            const auto parsed = parse_int(argv[++i]);
            if (!parsed.has_value())
            {
                std::cerr << "Invalid CAN ID value.\n";
                return false;
            }
            settings.md_ids.push_back(*parsed);
            continue;
        }
        if (arg == "--bus" && i + 1 < argc)
        {
            const std::string value = argv[++i];
            if (value == "usb")
            {
                settings.bus = mab::candleTypes::busTypes_t::USB;
            }
            else if (value == "spi")
            {
                settings.bus = mab::candleTypes::busTypes_t::SPI;
            }
            else
            {
                std::cerr << "Invalid bus '" << value << "'. Expected 'usb' or 'spi'.\n";
                return false;
            }
            continue;
        }
        if (arg == "--priority" && i + 1 < argc)
        {
            settings.priority = parse_int(argv[++i]);
            if (!settings.priority.has_value())
            {
                std::cerr << "Invalid priority value.\n";
                return false;
            }
            continue;
        }
        if (arg == "--mode" && i + 1 < argc)
        {
            const std::string value = argv[++i];
            if (value == "sequential")
            {
                settings.mode = BatchMode::Sequential;
            }
            else if (value == "async")
            {
                settings.mode = BatchMode::Async;
            }
            else
            {
                std::cerr << "Invalid mode '" << value
                          << "'. Expected 'sequential' or 'async'.\n";
                return false;
            }
            continue;
        }
        if (arg == "--samples" && i + 1 < argc)
        {
            const auto parsed = parse_int(argv[++i]);
            if (!parsed.has_value() || *parsed <= 0)
            {
                std::cerr << "Invalid samples value. Expected a positive integer.\n";
                return false;
            }
            settings.samples = *parsed;
            continue;
        }

        std::cerr << "Unknown or incomplete argument: " << arg << "\n";
        return false;
    }

    if (settings.md_ids.empty())
    {
        std::cerr << "At least one --id argument is required.\n";
        return false;
    }

    return true;
}

bool run_sample(const BatchMode mode,
                const std::vector<std::unique_ptr<mab::MD>> & mds,
                size_t & failed_requests)
{
    bool sample_ok = true;

    if (mode == BatchMode::Sequential)
    {
        for (const auto & md : mds)
        {
            const auto err = md->writeRegisters(md->m_mdRegisters.userGpioConfiguration);
            if (err != mab::MD::Error_t::OK)
            {
                sample_ok = false;
                ++failed_requests;
            }
        }
        return sample_ok;
    }

    std::vector<std::future<mab::MD::Error_t>> futures;
    futures.reserve(mds.size());
    for (const auto & md : mds)
    {
        futures.push_back(md->writeRegistersAsync(md->m_mdRegisters.userGpioConfiguration));
    }
    for (auto & future : futures)
    {
        if (future.get() != mab::MD::Error_t::OK)
        {
            sample_ok = false;
            ++failed_requests;
        }
    }
    return sample_ok;
}

Summary measure_latency(const Settings & settings,
                        const std::vector<std::unique_ptr<mab::MD>> & mds)
{
    Summary summary;
    double total_batch_us = 0.0;

    for (int sample = 0; sample < settings.samples; ++sample)
    {
        const auto start = std::chrono::high_resolution_clock::now();
        const bool sample_ok = run_sample(settings.mode, mds, summary.failed_requests);
        const auto end = std::chrono::high_resolution_clock::now();

        total_batch_us +=
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
        if (!sample_ok)
        {
            ++summary.failed_samples;
        }
    }

    summary.average_batch_us = total_batch_us / static_cast<double>(settings.samples);
    summary.batch_frequency_khz = 1000.0 / summary.average_batch_us;
    summary.average_per_drive_us =
        summary.average_batch_us / static_cast<double>(mds.size());
    summary.estimated_frame_time_us =
        kFrameTimePerTransferUs * static_cast<double>(mds.size());
    summary.estimated_host_overhead_us =
        summary.average_batch_us - summary.estimated_frame_time_us;
    summary.estimated_bus_utilization_pct =
        (summary.estimated_frame_time_us / summary.average_batch_us) * 100.0;

    return summary;
}

void print_summary(const Settings & settings, const Summary & summary)
{
    std::cout << "Latency batch summary\n";
    std::cout << "  bus: " << bus_name(settings.bus) << "\n";
    std::cout << "  mode: " << mode_name(settings.mode) << "\n";
    std::cout << "  drives:";
    for (const int id : settings.md_ids)
    {
        std::cout << " " << id;
    }
    std::cout << "\n";
    std::cout << "  samples: " << settings.samples << "\n";
    std::cout << "  average batch time: " << summary.average_batch_us << " us\n";
    std::cout << "  average batch frequency: " << summary.batch_frequency_khz << " kHz\n";
    std::cout << "  average per-drive slot: " << summary.average_per_drive_us << " us\n";
    std::cout << "  estimated frame time budget: " << summary.estimated_frame_time_us << " us\n";
    std::cout << "  estimated non-frame overhead: " << summary.estimated_host_overhead_us
              << " us\n";
    std::cout << "  estimated frame utilization: " << summary.estimated_bus_utilization_pct
              << "%\n";
    if (summary.failed_samples != 0 || summary.failed_requests != 0)
    {
        std::cout << "  failed samples: " << summary.failed_samples << "\n";
        std::cout << "  failed requests: " << summary.failed_requests << "\n";
    }
}

}  // namespace

int main(int argc, char ** argv)
{
    Settings settings;
    if (!parse_args(argc, argv, settings))
    {
        print_usage(argv[0]);
        return argc > 1 ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    if (settings.show_help)
    {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }

    if (settings.priority.has_value() && !set_fifo_priority(*settings.priority))
    {
        return EXIT_FAILURE;
    }

    mab::Candle * candle = nullptr;
    try
    {
        candle = mab::attachCandle(mab::CANdleDatarate_E::CAN_DATARATE_1M, settings.bus);
    }
    catch (const std::exception & error)
    {
        std::cerr << "Failed to attach CANdle: " << error.what() << "\n";
        return EXIT_FAILURE;
    }

    if (candle == nullptr)
    {
        std::cerr << "Failed to attach CANdle.\n";
        return EXIT_FAILURE;
    }

    std::vector<std::unique_ptr<mab::MD>> mds;
    mds.reserve(settings.md_ids.size());
    for (const int id : settings.md_ids)
    {
        auto md = std::make_unique<mab::MD>(static_cast<mab::canId_t>(id), candle);
        if (md->init() != mab::MD::Error_t::OK)
        {
            std::cerr << "MD " << id << " failed to initialize.\n";
            mab::detachCandle(candle);
            return EXIT_FAILURE;
        }
        md->m_mdRegisters.userGpioConfiguration = 0;
        mds.push_back(std::move(md));
    }

    const auto summary = measure_latency(settings, mds);
    print_summary(settings, summary);

    mab::detachCandle(candle);
    return EXIT_SUCCESS;
}
