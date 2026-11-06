cmake_minimum_required (VERSION 3.12)

set (EXTERNAL_LIBRARY_OPENNURBS_3RD_ZLIB_NAME "opennurbs_3rd_zlib" CACHE INTERNAL "EXTERNAL LIBRARY OpenNURBS 3RD ZLIB NAME")

project (${EXTERNAL_LIBRARY_OPENNURBS_3RD_ZLIB_NAME} C)

set (INCLUDE_DIRS .)

set (HEADERS
crc32.h
deflate.h
inffast.h
inffixed.h
inflate.h
inftrees.h
trees.h
zconf.h
zlib.h
zutil.h
)

set (SOURCES
adler32.c
compress.c
crc32.c
deflate.c
infback.c
inffast.c
inflate.c
inftrees.c
trees.c
uncompr.c
zutil.c
)
set (CMAKE_POSITION_INDEPENDENT_CODE ON) # -fPIC
add_library (${EXTERNAL_LIBRARY_OPENNURBS_3RD_ZLIB_NAME} STATIC
${SOURCES}
${HEADERS}
)
target_compile_definitions (${EXTERNAL_LIBRARY_OPENNURBS_3RD_ZLIB_NAME} PRIVATE Z_PREFIX MY_ZCALLOC)

target_include_directories (${EXTERNAL_LIBRARY_OPENNURBS_3RD_ZLIB_NAME} PUBLIC .)
