#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Applications/ti/tirtos_tivac_2_16_01_14/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/tidrivers_tivac_2_16_01_13/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/bios_6_45_02_31/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/ndk_2_25_00_09/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/uia_2_00_05_50/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/ns_1_11_00_10/packages;/Users/fazalmahmood/ECE_4437_F19/Final_Robot_Project_Fall_19/.config
override XDCROOT = /Users/fazalmahmood/ti/xdctools_3_31_03_43_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Applications/ti/tirtos_tivac_2_16_01_14/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/tidrivers_tivac_2_16_01_13/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/bios_6_45_02_31/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/ndk_2_25_00_09/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/uia_2_00_05_50/packages;/Applications/ti/tirtos_tivac_2_16_01_14/products/ns_1_11_00_10/packages;/Users/fazalmahmood/ECE_4437_F19/Final_Robot_Project_Fall_19/.config;/Users/fazalmahmood/ti/xdctools_3_31_03_43_core/packages;..
HOSTOS = MacOS
endif
