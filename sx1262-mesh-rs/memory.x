MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

/* This is the magic for RTIC/Cortex-M */
_stext = ADDR(FLASH) + 0x0; 

SECTIONS {
    .boot2 : {
        KEEP(*(.boot2))
    } > BOOT2
} INSERT BEFORE .text;