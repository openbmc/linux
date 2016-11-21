/*
 *  ast_pwm_techo_h  
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
struct ast_pwm_driver_data {
	u32 (*get_pwm_clock)(void);
};

