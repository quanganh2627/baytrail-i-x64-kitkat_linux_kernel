#ifndef _ASM_X86_INTEL_MID_OSIP_H_
#define _ASM_X86_INTEL_MID_OSIP_H_

extern int intel_mid_osip_read_oshob(u8 *data, int len, int offset);
extern int intel_mid_osip_write_osnib(u8 *data, int len, int offset, u32 mask);
extern int intel_mid_osip_write_osnib_rr(u8 rr);
extern int intel_mid_osip_read_osnib_rr(u8 *rr);

#endif
