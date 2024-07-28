


include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/app
	${CMAKE_CURRENT_SOURCE_DIR}/ip/inc
	${CMAKE_CURRENT_SOURCE_DIR}/ip/ll/ble/src
	${CMAKE_CURRENT_SOURCE_DIR}/ip/ll/ble/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/ll/em/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/ll/sch/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/ll/data_path
	${CMAKE_CURRENT_SOURCE_DIR}/ip/hci/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/hl/inc
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/hl/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/hl/src/gap/gapc
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/profiles/dis/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/profiles/bas/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/profiles/hogp/api
                ${CMAKE_CURRENT_SOURCE_DIR}/ip/host/profiles/gatv/api
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/ahi/api 
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/thirdparty/md5/inc
	${CMAKE_CURRENT_SOURCE_DIR}/ip/host/thirdparty/yxml/inc
	${CMAKE_CURRENT_SOURCE_DIR}/modules/security/api
	${CMAKE_CURRENT_SOURCE_DIR}/modules/common/api
	${CMAKE_CURRENT_SOURCE_DIR}/modules/dbg/api
	${CMAKE_CURRENT_SOURCE_DIR}/modules/h4tl/api
	${CMAKE_CURRENT_SOURCE_DIR}/modules/ke/api
	${CMAKE_CURRENT_SOURCE_DIR}/modules/nvds/api 
	${CMAKE_CURRENT_SOURCE_DIR}/modules/rf/api
	${CMAKE_CURRENT_SOURCE_DIR}/modules/rwip/api
	${CMAKE_CURRENT_SOURCE_DIR}/plf/arch_lib/src/sbc 
	${CMAKE_CURRENT_SOURCE_DIR}/plf/data_path 
	${CMAKE_CURRENT_SOURCE_DIR}/plf/include
	${CMAKE_CURRENT_SOURCE_DIR}/plf/plf
	${CMAKE_CURRENT_SOURCE_DIR}/../freertos/include
)

add_compile_options( -Wchar-subscripts -Wformat -Wuninitialized -Wno-stringop-truncation -Wno-restrict -Winit-self -Wignored-qualifiers -Wswitch-default -Wunused -Wundef -Wclobbered -Wempty-body -Wignored-qualifiers -Wmissing-parameter-type -Wold-style-declaration -Woverride-init -Wuninitialized -Wmissing-field-initializers -Wunused-but-set-parameter -Wsign-compare -Werror  -mabi=ilp32 -mcmodel=medlow)

add_definitions(-DCFG_APP)
add_definitions(-DCFG_APP_PRF)
add_definitions(-DCFG_APP_SEC)
add_definitions(-DCFG_APP_HID)               
add_definitions(-DCFG_APP_DIS)
add_definitions(-DCFG_PRF_BASS)
add_definitions(-DCFG_APP_GATV)
add_definitions(-DCFG_PRF_GATVS)
add_definitions(-DCFG_BLE_EMB)
#add_definitions(-DCFG_CONLESS_CTE_RX)
#add_definitions(-DCFG_CONLESS_CTE_TX)
add_definitions(-DCFG_BLE_ISO_HW_SUPPORT_NOT_PRESENT)
add_definitions(-DCFG_BLE_HOST)
add_definitions(-DCFG_CON=10)
add_definitions(-DCFG_HL_MSG_API)
add_definitions(-DCFG_GATT_CLI)
add_definitions(-DCFG_HOST_KEY_MGR)
add_definitions(-DCFG_MAX_LE_MTU=2048)
add_definitions(-DCFG_ATT_VAL_MAX=2048)
add_definitions(-DCFG_L2CAP_COC_CHAN_PER_CON_NB=10)
add_definitions(-DCFG_L2CAP_CHAN_IN_ENV_NB=20)
add_definitions(-DCFG_PRF_DISS)
add_definitions( -DCFG_PRF_BASS)
add_definitions(-DCFG_PRF_HOGPD)
add_definitions(-DCFG_PRF)
add_definitions(-DCFG_NB_PRF=10)
#add_definitions(-DCFG_AHITL)
add_definitions(-DCFG_NVDS)
#add_definitions(-DCFG_NVDS_RAM)
add_definitions(-DCFG_RF_XRC443)
add_definitions(-DCFG_MODEM_LR)
add_definitions(-DCFG_STATIC)
add_definitions(-DCFG_ALLROLES)
add_definitions(-DCFG_LE_PWR_CTRL)
#add_definitions(-DCFG_AOD)
add_definitions(-DCFG_ISO_CON=3)
add_definitions(-DCFG_RAL=3)
add_definitions(-DCFG_ACT=12)
add_definitions(-DCFG_GAIA)
add_definitions(-DCFG_GPIO)
add_definitions(-DCFG_ECC_P256_SUPPORT)
#add_definitions(-DCFG_DBG)


add_definitions(-DCFG_ROM_VT)
add_definitions(-DCFG_RTOS)


################### ceva_ble #########################

file(GLOB_RECURSE ceva_ble ${CMAKE_CURRENT_SOURCE_DIR}/ip/ll/*.c)
list (APPEND DIR_CEVA_BLE_SRCS ${ceva_ble})

file(GLOB_RECURSE ceva_ble ${CMAKE_CURRENT_SOURCE_DIR}/vtPatch/JumpTable.c)
list (APPEND DIR_CEVA_BLE_SRCS ${ceva_ble})

################### ceva_host #########################

file(GLOB_RECURSE ceva_host ${CMAKE_CURRENT_SOURCE_DIR}/app/*.c)
list (APPEND DIR_CEVA_HOST_SRCS ${ceva_host})

file(GLOB_RECURSE ceva_host ${CMAKE_CURRENT_SOURCE_DIR}/ip/hci/*.c)
list (APPEND DIR_CEVA_HOST_SRCS ${ceva_host})

file(GLOB_RECURSE ceva_host ${CMAKE_CURRENT_SOURCE_DIR}/ip/host/*.c)
list (APPEND DIR_CEVA_HOST_SRCS ${ceva_host})

file(GLOB_RECURSE ceva_host ${CMAKE_CURRENT_SOURCE_DIR}/modules/*.c)
list (APPEND DIR_CEVA_HOST_SRCS ${ceva_host})

file(GLOB_RECURSE ceva_host ${CMAKE_CURRENT_SOURCE_DIR}/plf/*.c)
list (APPEND DIR_CEVA_HOST_SRCS ${ceva_host})

file(GLOB_RECURSE ceva_host ${CMAKE_CURRENT_SOURCE_DIR}/vtPatch/vtPatchs.c)
list (APPEND DIR_CEVA_HOST_SRCS ${ceva_host})


################### ceva_patchs #########################
file(GLOB_RECURSE ceva_patch ${CMAKE_CURRENT_SOURCE_DIR}/ip/ll/*_vtpatch.c)
list (REMOVE_ITEM DIR_CEVA_BLE_SRCS ${ceva_patch})
list (APPEND DIR_CEVA_HOST_SRCS ${ceva_patch})


################### ceva_libc #########################
file(GLOB_RECURSE ceva_libc ${CMAKE_CURRENT_SOURCE_DIR}/plf/arch_lib/src/string/*.c)
list (APPEND DIR_CEVA_BLE_SRCS ${ceva_libc})
list (REMOVE_ITEM DIR_CEVA_HOST_SRCS ${ceva_libc})

message("ceva lib compiled file=" ${DIR_CEVA_BLE_SRCS})
add_library (ceva_ble STATIC ${DIR_CEVA_BLE_SRCS})
#message("ceva_ble file =" ${DIR_CEVA_BLE_SRCS})

message("ceva host compiled file=" ${DIR_CEVA_HOST_SRCS})
add_library (ceva_host STATIC ${DIR_CEVA_HOST_SRCS})
#message("ceva_host file =" ${DIR_CEVA_HOST_SRCS})


target_compile_definitions(ceva_ble PUBLIC -DCFG_ROM_VT_IMPORT)

