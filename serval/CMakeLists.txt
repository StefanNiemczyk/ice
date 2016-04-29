cmake_minimum_required(VERSION 2.8.3)
project(serval C)

find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serval
)

set(CMAKE_C_COMPILER "gcc")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DSYSCONFDIR=\"\\\"/usr/local/etc\"\\\" -DLOCALSTATEDIR=\"\\\"/usr/local/var\"\\\"")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPACKAGE_NAME=\"servald\" -DPACKAGE_TARNAME=\"servald\" -DPACKAGE_VERSION=\"0.9\" -DPACKAGE_STRING=\"servald\ 0.9\" --param=ssp-buffer-size=4  -D_GNU_SOURCE -DSQLITE_THREADSAFE=0 -DSQLITE_OMIT_DATETIME_FUNCS -DSQLITE_OMIT_COMPILEOPTION_DIAGS -DSQLITE_OMIT_DEPRECATED -DSQLITE_OMIT_LOAD_EXTENSION -DSQLITE_OMIT_VIRTUALTABLE -DSQLITE_OMIT_AUTHORIZATION -fPIC -Wall -DSHA2_USE_INTTYPES_H -D_XOPEN_SOURCE=600 -D_XOPEN_SOURCE_EXTENDED=1 -DHAVE_FUNC_ATTRIBUTE_ALIGNED=1 -DHAVE_FUNC_ATTRIBUTE_ALLOC_SIZE=1 -DHAVE_FUNC_ATTRIBUTE_ERROR=1 -DHAVE_FUNC_ATTRIBUTE_FORMAT=1 -DHAVE_FUNC_ATTRIBUTE_MALLOC=1 -DHAVE_FUNC_ATTRIBUTE_UNUSED=1 -DHAVE_FUNC_ATTRIBUTE_USED=1 -DHAVE_VAR_ATTRIBUTE_SECTION=1 -DHAVE_SYS_TYPES_H=1 -DHAVE_SYS_STAT_H=1 -DHAVE_STRING_H=1 -DHAVE_MEMORY_H=1 -DHAVE_STRINGS_H=1 -DHAVE_INTTYPES_H=1 -DHAVE_STDINT_H=1 -DHAVE_UNISTD_H=1 -DHAVE_MATH_H=1 -DHAVE_FLOAT_H=1 -DHAVE_LIBRT=1 -DHAVE_BCOPY=1 -DHAVE_BZERO=1 -DHAVE_BCMP=1 -DHAVE_LSEEK64=1 -DHAVE_OFF64_T=1 -DSIZEOF_OFF_T=8 -DHAVE_STDIO_H=1 -DHAVE_ERRNO_H=1 -DHAVE_STDLIB_H=1 -DHAVE_STRINGS_H=1 -DHAVE_UNISTD_H=1 -DHAVE_STRING_H=1 -DHAVE_ARPA_INET_H=1 -DHAVE_SYS_SOCKET_H=1 -DHAVE_SYS_MMAN_H=1 -DHAVE_SYS_TIME_H=1 -DHAVE_SYS_STATVFS_H=1 -DHAVE_SYS_STAT_H=1 -DHAVE_SYS_VFS_H=1 -DHAVE_POLL_H=1 -DHAVE_NETDB_H=1 -DHAVE_LINUX_IOCTL_H=1 -DHAVE_LINUX_NETLINK_H=1 -DHAVE_LINUX_RTNETLINK_H=1 -DHAVE_NET_IF_H=1 -DHAVE_NETINET_IN_H=1 -DHAVE_IFADDRS_H=1 -DHAVE_NET_ROUTE_H=1 -DHAVE_SIGNAL_H=1 -DHAVE_SYS_SOCKET_H=1 -DHAVE_LINUX_IF_H=1 -DUSE_ABSTRACT_NAMESPACE=1 -DHAVE_SINF=1 -DHAVE_COSF=1 -DHAVE_TANF=1 -DHAVE_ASINF=1 -DHAVE_ACOSF=1 -DHAVE_ATANF=1 -DHAVE_ATAN2F=1 -DHAVE_CEILF=1 -DHAVE_FLOORF=1 -DHAVE_POWF=1 -DHAVE_EXPF=1 -DHAVE_LOGF=1 -DHAVE_LOG10F=1")

include_directories(
	serval-dna/nacl/include
	serval-dna/sqlite-amalgamation-3070900
)

add_library(serval STATIC
        serval-dna/cli.c
        serval-dna/conf.c
        serval-dna/conf_om.c
        serval-dna/conf_parse.c
        serval-dna/conf_schema.c
        serval-dna/console.c
        serval-dna/dataformats.c
        serval-dna/fdqueue.c
        serval-dna/limit.c
        serval-dna/logMessage.c
        serval-dna/log_util.c
        serval-dna/mem.c
        serval-dna/net.c
        serval-dna/os.c
        serval-dna/performance_timing.c
        serval-dna/randombytes.c
        serval-dna/rotbuf.c
        serval-dna/sighandlers.c
        serval-dna/socket.c
        serval-dna/srandomdev.c
        serval-dna/strbuf.c
        serval-dna/strbuf_helpers.c
        serval-dna/str.c
        serval-dna/strlcpy.c
        serval-dna/sha2.c
        serval-dna/uuid.c
        serval-dna/xprintf.c
        serval-dna/whence.c
	serval-dna/mdp_client.c
	serval-dna/instance.c
	serval-dna/log_stderr.c
	serval-dna/log_context.c
	serval-dna/log_on_config_change.c
)

add_custom_command(TARGET serval PRE_BUILD COMMAND cp "${PROJECT_SOURCE_DIR}/serval-dna/constants.h" "${PROJECT_SOURCE_DIR}/include")


