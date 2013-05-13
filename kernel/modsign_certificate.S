#include <linux/export.h>

#define GLOBAL(name)	\
	.globl VMLINUX_SYMBOL(name);	\
	VMLINUX_SYMBOL(name):

	.section ".init.data","aw"

GLOBAL(modsign_certificate_list)
#ifdef CONFIG_MODULE_SIG_BUILTIN_CERTS
	.incbin "signing_key.x509"
	.incbin "extra_certificates"
#endif
GLOBAL(modsign_certificate_list_end)
