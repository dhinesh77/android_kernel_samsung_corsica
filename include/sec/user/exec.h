/**
 * COPYRIGHT (C)  SAMSUNG Electronics CO., LTD (Suwon, Korea). 2009
 * All rights are reserved. Reproduction and redistiribution in whole or
 * in part is prohibited without the written consent of the copyright owner.
 */

/**
 * User execution device driver on panic.
 *
 * On device init time it was registered to panic handler.
 * When panic handler chain was called, * this handler execute some user 
 * processes.
 * 
 * @author kumhyun.cho@samsung.com
 * @since 2012.03.19
 */

#ifndef __SEC_USER_EXEC_H__
#define __SEC_USER_EXEC_H__

int sec_user_exec_handler(struct notifier_block* nb, unsigned long l, void* buf);

#endif /* __SEC_USER_EXEC_H__ */
