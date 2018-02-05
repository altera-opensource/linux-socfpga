/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _XT_HASHLIMIT_H
#define _XT_HASHLIMIT_H

#include <uapi/linux/netfilter/xt_hashlimit.h>

#define XT_HASHLIMIT_ALL (XT_HASHLIMIT_HASH_DIP | XT_HASHLIMIT_HASH_DPT | \
			  XT_HASHLIMIT_HASH_SIP | XT_HASHLIMIT_HASH_SPT | \
			  XT_HASHLIMIT_INVERT | XT_HASHLIMIT_BYTES |\
			  XT_HASHLIMIT_RATE_MATCH)
#endif /*_XT_HASHLIMIT_H*/
