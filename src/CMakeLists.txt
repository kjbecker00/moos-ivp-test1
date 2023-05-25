# ##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt DATE:        2010/09/07
# 2020/05/09 minor mods DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend
# source directory
# ##############################################################################

# ============================================================================
# Add the libraries in the current directory to the include path
# ============================================================================
file(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
include_directories(${LOCAL_LIBRARY_DIRS})

# ============================================================================
# List the subdirectories to build...
# ============================================================================
add_subdirectory(pExampleApp)
add_subdirectory(pXRelayTest)
add_subdirectory(pGenRescue)
add_subdirectory(pSearchPath)
add_subdirectory(pSwimSearch)
add_subdirectory(pNodeIdentifier)
add_subdirectory(pScoutMgr)
add_subdirectory(lib_escape_windup_bhv)

# ##############################################################################
# END of CMakeLists.txt
# ##############################################################################
