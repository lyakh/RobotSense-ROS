/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

struct robot_state {
	int32_t		angle;
	uint32_t	distance;
} __attribute__((packed));

struct robot_control {
	int32_t		angle;
	int32_t		advance;
	int32_t		turn;
} __attribute__((packed));

namespace robot_sense {
int client_send(struct robot_state *state);
}

#endif
