#ifndef _BOARD_ID_H
#define _BOARD_ID_H

#define BOARD_SOFIA3G_SVB1
#define BOARD_SOFIA3G_SVB2		(1536)
#define BOARD_SOFIA3G_MRD_P0
#define BOARD_SOFIA3G_MRD_7S		(15)
#define BOARD_SOFIA3G_MRD_5S		(13)

int sofia_board_is(id);
unsigned int sofia_get_board_id(void);

#endif
