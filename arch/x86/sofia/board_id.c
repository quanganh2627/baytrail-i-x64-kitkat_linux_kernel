#include <linux/kernel.h>
#include <linux/init.h>

static unsigned int board_id = 0xffff;

static int __init sofia_board_id_setup(char *str)
{
	get_option(&str, &board_id);
	pr_info("Sofia Board ID is 0x%x\n", board_id);

	return 0;
}
__setup("board_id=", sofia_board_id_setup);

int sofia_board_is(unsigned int id)
{
	return (board_id == id);
}

unsigned int sofia_get_board_id(void)
{
	return board_id;
}
