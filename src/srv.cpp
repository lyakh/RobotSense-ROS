/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2019, Guennadi Liakhovetski
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/evp.h>

#include "robot_data.h"
#include "ros-listen.h"

namespace robot_sense {

static SSL *instance_ssl;

int client_send(struct robot_state *state)
{
	state->angle = htonl(state->angle);
	state->distance = htonl(state->distance);

	int ret = SSL_write(instance_ssl, state, sizeof(*state));
	if (!ret)
		ret = -EINVAL;

	return ret;
}

class ssl_link {
public:
	ssl_link();
	~ssl_link();
	int create_socket(int port);
	int create_context();
	int configure_context();
	int link();
	void release();
	int read(struct robot_control *control);

private:
	int sock;
	int client;
	SSL_CTX *ctx;
	SSL *ssl;
};

ssl_link::ssl_link()
{ 
	SSL_load_error_strings();	
	OpenSSL_add_ssl_algorithms();
}

ssl_link::~ssl_link()
{
	close(sock);
	SSL_CTX_free(ctx);
	EVP_cleanup();
}

int ssl_link::create_socket(int port)
{
	struct sockaddr_in addr;
	int ret;

	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("Unable to create socket");
		return sock;
	}

	ret = bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	if (ret < 0) {
		perror("Unable to bind");
		return ret;
	}

	ret = listen(sock, 1);
	if (ret < 0) {
		perror("Unable to listen");
		return ret;
	}

	return 0;
}

int ssl_link::create_context()
{
	const SSL_METHOD *method = SSLv23_server_method();

	ctx = SSL_CTX_new(method);
	if (!ctx) {
		perror("Unable to create SSL context");
		ERR_print_errors_fp(stderr);
		return -ENOMEM;
	}

	return 0;
}

int ssl_link::configure_context()
{
	int ret;

	SSL_CTX_set_ecdh_auto(ctx, 1);

	/* Set the key and cert */
	ret = SSL_CTX_use_certificate_file(ctx, "/etc/ros-project/car/robot.crt", SSL_FILETYPE_PEM);
	if (ret <= 0) {
		ERR_print_errors_fp(stderr);
		return ret < 0 ? ret : -EINVAL;
	}

	ret = SSL_CTX_use_PrivateKey_file(ctx, "/etc/ros-project/car/robot.key", SSL_FILETYPE_PEM);
	if (ret <= 0 ) {
		ERR_print_errors_fp(stderr);
		return ret < 0 ? ret : -EINVAL;
	}

	return 0;
}

int ssl_link::link()
{
	struct sockaddr_in addr;
	uint len = sizeof(addr);

	client = accept(sock, (struct sockaddr*)&addr, &len);
	if (client < 0) {
		perror("Unable to accept");
		return client;
	}

	ssl = SSL_new(ctx);
	SSL_set_fd(ssl, client);

	int ret = SSL_accept(ssl);
	if (ret <= 0) {
		ERR_print_errors_fp(stderr);
		return ret < 0 ? ret : -EINVAL;
	}

	instance_ssl = ssl;

	return 0;
}

void ssl_link::release()
{
	SSL_free(ssl);
	close(client);
}

int ssl_link::read(struct robot_control *control)
{
	int ret = SSL_read(ssl, control, sizeof(*control));
	if (ret <= 0) {
		fprintf(stderr, "ssl_read: %d\n", ret);
		return ret ? : -EINVAL;
	}

	control->angle = ntohl(control->angle);
	control->advance = ntohl(control->advance);
	control->turn = ntohl(control->turn);

	return ret;
}

}

int main(int argc, char *argv[])
{
	SSL_CTX *ctx;
	robot_sense::ssl_link *ssl = new robot_sense::ssl_link();
	int ret = ssl->create_context();
	if (ret < 0)
		exit(EXIT_FAILURE);

	ret = ssl->configure_context();
	if (ret < 0)
		exit(EXIT_FAILURE);

	ros::init(argc, argv, "ctl_srv");

	ret = ssl->create_socket(4433);
	if (ret < 0)
		exit(EXIT_FAILURE);

	robot_sense::listener *listener = new robot_sense::listener(argc, argv);

	/* Handle connections */
	for (;;) {
		if (!ssl->link())
			for (;;) {
				struct robot_control control;
				int ret = ssl->read(&control);

				if (ret < 0)
					control.angle = 0;

				int err = listener->control(&control);

				if (err < 0 || ret < 0)
					break;

				fprintf(stderr, "%s(): %d: %d, %d, %d\n", __func__, ret,
					control.angle, control.advance, control.turn);
			}

		ssl->release();
	}
}
