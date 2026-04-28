#pragma once

#include <string>

#include "MD.hpp"

namespace mab_rehab
{

/// Upload the contents of an MD .cfg file (INI-style, same format as
/// candletool) to the drive. Cold-path; may block for hundreds of ms
/// per register write, so must never be called from the RT thread.
///
/// Mirrors the behaviour of `candletool md config upload <path>`.
///
/// Returns true on success; on failure, error_out is populated with a
/// human-readable reason and the drive is left in an indeterminate state
/// (caller should disable the drive before re-trying).
bool upload_md_config(
  mab::MD & md,
  const std::string & path,
  std::string & error_out,
  bool save_to_flash);

}  // namespace mab_rehab
