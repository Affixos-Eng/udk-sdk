SECTION_DATA_PROLOGUE(.dw_drivers,,)
{
	. = ALIGN(4);
	__dw_drivers_load_start__ = .;
    __dw_drivers_start__ = .;
    __dw_drivers_start =   __dw_drivers_start__;
    KEEP(*(.dw_drivers .dw_drivers.*))
	. = ALIGN(4);
} GROUP_LINK_IN(ROMABLE_REGION)

__dw_drivers_end__ = __dw_drivers_start__ + SIZEOF(.dw_drivers);
__dw_drivers_size__ = SIZEOF(.dw_drivers);
__dw_drivers_end =   __dw_drivers_end__;
__dw_drivers_load_end__ = __dw_drivers_end__;

