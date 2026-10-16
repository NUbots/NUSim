# Fast-DDS 2.13.6 attempts to apply an affinity tag even when the default is 0 (no requested affinity). Apple Silicon
# rejects the call; its logging thread then logs its own setup failure and deadlocks. Match the Linux implementation's
# no-op for affinity 0, without changing DDS versions or wire behaviour.
set(_osx_threading "${FASTDDS_SOURCE}/src/cpp/utils/threading/threading_osx.ipp")
file(READ "${_osx_threading}" _osx_original)
string(REPLACE "if (affinity <=" "if (affinity != 0 && affinity <=" _osx_fixed "${_osx_original}")
if(NOT _osx_fixed STREQUAL _osx_original)
  file(WRITE "${_osx_threading}" "${_osx_fixed}")
endif()
