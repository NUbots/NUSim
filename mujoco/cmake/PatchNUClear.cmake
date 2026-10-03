# The pinned NUClear commit omits <> after two dependent template calls. Clang 21 rejects this syntax. Fix only those
# calls in the fetched source, preserving the dependency version and behaviour.
set(_bind_fusion "${nuclear_SOURCE_DIR}/src/dsl/fusion/BindFusion.hpp")
file(READ "${_bind_fusion}" _bind_original)
string(REPLACE ">::template call(" ">::template call<>(" _bind_fixed "${_bind_original}")
if(NOT _bind_fixed STREQUAL _bind_original)
  file(WRITE "${_bind_fusion}" "${_bind_fixed}")
endif()
