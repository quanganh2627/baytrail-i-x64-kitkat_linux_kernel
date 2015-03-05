#ifndef _BOARD_ID_H
#define _BOARD_ID_H

#define BOARD_SOFIA3G_SVB1
#define BOARD_SOFIA3G_SVB2		(1536)
#define BOARD_SOFIA3G_MRD_P0
#define BOARD_SOFIA3G_MRD_7S		(15)
#define BOARD_SOFIA3G_MRD_5S		(13)
/* MRD 5S quad band variant has different RF,
 * other components are same
 */
#define BOARD_SOFIA3G_MRD_5S_QB		(6)

int sofia_board_is(unsigned int id);
unsigned int sofia_get_board_id(void);

#endif
