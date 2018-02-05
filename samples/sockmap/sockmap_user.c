/* Copyright (c) 2017 Covalent IO, Inc. http://covalent.io
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <time.h>

#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>

#include <linux/netlink.h>
#include <linux/socket.h>
#include <linux/sock_diag.h>
#include <linux/bpf.h>
#include <linux/if_link.h>
#include <assert.h>
#include <libgen.h>

#include <getopt.h>

#include "../bpf/bpf_load.h"
#include "../bpf/bpf_util.h"
#include "../bpf/libbpf.h"

int running;
void running_handler(int a);

/* randomly selected ports for testing on lo */
#define S1_PORT 10000
#define S2_PORT 10001

/* global sockets */
int s1, s2, c1, c2, p1, p2;

static const struct option long_options[] = {
	{"help",	no_argument,		NULL, 'h' },
	{"cgroup",	required_argument,	NULL, 'c' },
	{"rate",	required_argument,	NULL, 'r' },
	{"verbose",	no_argument,		NULL, 'v' },
	{"iov_count",	required_argument,	NULL, 'i' },
	{"length",	required_argument,	NULL, 'l' },
	{"test",	required_argument,	NULL, 't' },
	{0, 0, NULL, 0 }
};

static void usage(char *argv[])
{
	int i;

	printf(" Usage: %s --cgroup <cgroup_path>\n", argv[0]);
	printf(" options:\n");
	for (i = 0; long_options[i].name != 0; i++) {
		printf(" --%-12s", long_options[i].name);
		if (long_options[i].flag != NULL)
			printf(" flag (internal value:%d)\n",
				*long_options[i].flag);
		else
			printf(" -%c\n", long_options[i].val);
	}
	printf("\n");
}

static int sockmap_init_sockets(void)
{
	int i, err, one = 1;
	struct sockaddr_in addr;
	int *fds[4] = {&s1, &s2, &c1, &c2};

	s1 = s2 = p1 = p2 = c1 = c2 = 0;

	/* Init sockets */
	for (i = 0; i < 4; i++) {
		*fds[i] = socket(AF_INET, SOCK_STREAM, 0);
		if (*fds[i] < 0) {
			perror("socket s1 failed()");
			return errno;
		}
	}

	/* Allow reuse */
	for (i = 0; i < 2; i++) {
		err = setsockopt(*fds[i], SOL_SOCKET, SO_REUSEADDR,
				 (char *)&one, sizeof(one));
		if (err) {
			perror("setsockopt failed()");
			return errno;
		}
	}

	/* Non-blocking sockets */
	for (i = 0; i < 2; i++) {
		err = ioctl(*fds[i], FIONBIO, (char *)&one);
		if (err < 0) {
			perror("ioctl s1 failed()");
			return errno;
		}
	}

	/* Bind server sockets */
	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	addr.sin_port = htons(S1_PORT);
	err = bind(s1, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0) {
		perror("bind s1 failed()\n");
		return errno;
	}

	addr.sin_port = htons(S2_PORT);
	err = bind(s2, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0) {
		perror("bind s2 failed()\n");
		return errno;
	}

	/* Listen server sockets */
	addr.sin_port = htons(S1_PORT);
	err = listen(s1, 32);
	if (err < 0) {
		perror("listen s1 failed()\n");
		return errno;
	}

	addr.sin_port = htons(S2_PORT);
	err = listen(s2, 32);
	if (err < 0) {
		perror("listen s1 failed()\n");
		return errno;
	}

	/* Initiate Connect */
	addr.sin_port = htons(S1_PORT);
	err = connect(c1, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0 && errno != EINPROGRESS) {
		perror("connect c1 failed()\n");
		return errno;
	}

	addr.sin_port = htons(S2_PORT);
	err = connect(c2, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0 && errno != EINPROGRESS) {
		perror("connect c2 failed()\n");
		return errno;
	} else if (err < 0) {
		err = 0;
	}

	/* Accept Connecrtions */
	p1 = accept(s1, NULL, NULL);
	if (p1 < 0) {
		perror("accept s1 failed()\n");
		return errno;
	}

	p2 = accept(s2, NULL, NULL);
	if (p2 < 0) {
		perror("accept s1 failed()\n");
		return errno;
	}

	printf("connected sockets: c1 <-> p1, c2 <-> p2\n");
	printf("cgroups binding: c1(%i) <-> s1(%i) - - - c2(%i) <-> s2(%i)\n",
		c1, s1, c2, s2);
	return 0;
}

struct msg_stats {
	size_t bytes_sent;
	size_t bytes_recvd;
	struct timespec start;
	struct timespec end;
};

static int msg_loop(int fd, int iov_count, int iov_length, int cnt,
		    struct msg_stats *s, bool tx)
{
	struct msghdr msg = {0};
	int err, i, flags = MSG_NOSIGNAL;
	struct iovec *iov;

	iov = calloc(iov_count, sizeof(struct iovec));
	if (!iov)
		return errno;

	for (i = 0; i < iov_count; i++) {
		char *d = calloc(iov_length, sizeof(char));

		if (!d) {
			fprintf(stderr, "iov_count %i/%i OOM\n", i, iov_count);
			goto out_errno;
		}
		iov[i].iov_base = d;
		iov[i].iov_len = iov_length;
	}

	msg.msg_iov = iov;
	msg.msg_iovlen = iov_count;

	if (tx) {
		clock_gettime(CLOCK_MONOTONIC, &s->start);
		for (i = 0; i < cnt; i++) {
			int sent = sendmsg(fd, &msg, flags);

			if (sent < 0) {
				perror("send loop error:");
				goto out_errno;
			}
			s->bytes_sent += sent;
		}
		clock_gettime(CLOCK_MONOTONIC, &s->end);
	} else {
		int slct, recv, max_fd = fd;
		struct timeval timeout;
		float total_bytes;
		fd_set w;

		total_bytes = (float)iov_count * (float)iov_length * (float)cnt;
		err = clock_gettime(CLOCK_MONOTONIC, &s->start);
		if (err < 0)
			perror("recv start time: ");
		while (s->bytes_recvd < total_bytes) {
			timeout.tv_sec = 1;
			timeout.tv_usec = 0;

			/* FD sets */
			FD_ZERO(&w);
			FD_SET(fd, &w);

			slct = select(max_fd + 1, &w, NULL, NULL, &timeout);
			if (slct == -1) {
				perror("select()");
				clock_gettime(CLOCK_MONOTONIC, &s->end);
				goto out_errno;
			} else if (!slct) {
				fprintf(stderr, "unexpected timeout\n");
				errno = -EIO;
				clock_gettime(CLOCK_MONOTONIC, &s->end);
				goto out_errno;
			}

			recv = recvmsg(fd, &msg, flags);
			if (recv < 0) {
				if (errno != EWOULDBLOCK) {
					clock_gettime(CLOCK_MONOTONIC, &s->end);
					perror("recv failed()\n");
					goto out_errno;
				}
			}

			s->bytes_recvd += recv;
		}
		clock_gettime(CLOCK_MONOTONIC, &s->end);
	}

	for (i = 0; i < iov_count; i++)
		free(iov[i].iov_base);
	free(iov);
	return 0;
out_errno:
	for (i = 0; i < iov_count; i++)
		free(iov[i].iov_base);
	free(iov);
	return errno;
}

static float giga = 1000000000;

static inline float sentBps(struct msg_stats s)
{
	return s.bytes_sent / (s.end.tv_sec - s.start.tv_sec);
}

static inline float recvdBps(struct msg_stats s)
{
	return s.bytes_recvd / (s.end.tv_sec - s.start.tv_sec);
}

static int sendmsg_test(int iov_count, int iov_buf, int cnt,
			int verbose, bool base)
{
	float sent_Bps = 0, recvd_Bps = 0;
	int rx_fd, txpid, rxpid, err = 0;
	struct msg_stats s = {0};
	int status;

	errno = 0;

	if (base)
		rx_fd = p1;
	else
		rx_fd = p2;

	rxpid = fork();
	if (rxpid == 0) {
		err = msg_loop(rx_fd, iov_count, iov_buf, cnt, &s, false);
		if (err)
			fprintf(stderr,
				"msg_loop_rx: iov_count %i iov_buf %i cnt %i err %i\n",
				iov_count, iov_buf, cnt, err);
		shutdown(p2, SHUT_RDWR);
		shutdown(p1, SHUT_RDWR);
		if (s.end.tv_sec - s.start.tv_sec) {
			sent_Bps = sentBps(s);
			recvd_Bps = recvdBps(s);
		}
		fprintf(stdout,
			"rx_sendmsg: TX: %zuB %fB/s %fGB/s RX: %zuB %fB/s %fGB/s\n",
			s.bytes_sent, sent_Bps, sent_Bps/giga,
			s.bytes_recvd, recvd_Bps, recvd_Bps/giga);
		exit(1);
	} else if (rxpid == -1) {
		perror("msg_loop_rx: ");
		return errno;
	}

	txpid = fork();
	if (txpid == 0) {
		err = msg_loop(c1, iov_count, iov_buf, cnt, &s, true);
		if (err)
			fprintf(stderr,
				"msg_loop_tx: iov_count %i iov_buf %i cnt %i err %i\n",
				iov_count, iov_buf, cnt, err);
		shutdown(c1, SHUT_RDWR);
		if (s.end.tv_sec - s.start.tv_sec) {
			sent_Bps = sentBps(s);
			recvd_Bps = recvdBps(s);
		}
		fprintf(stdout,
			"tx_sendmsg: TX: %zuB %fB/s %f GB/s RX: %zuB %fB/s %fGB/s\n",
			s.bytes_sent, sent_Bps, sent_Bps/giga,
			s.bytes_recvd, recvd_Bps, recvd_Bps/giga);
		exit(1);
	} else if (txpid == -1) {
		perror("msg_loop_tx: ");
		return errno;
	}

	assert(waitpid(rxpid, &status, 0) == rxpid);
	assert(waitpid(txpid, &status, 0) == txpid);
	return err;
}

static int forever_ping_pong(int rate, int verbose)
{
	struct timeval timeout;
	char buf[1024] = {0};
	int sc;

	timeout.tv_sec = 10;
	timeout.tv_usec = 0;

	/* Ping/Pong data from client to server */
	sc = send(c1, buf, sizeof(buf), 0);
	if (sc < 0) {
		perror("send failed()\n");
		return sc;
	}

	do {
		int s, rc, i, max_fd = p2;
		fd_set w;

		/* FD sets */
		FD_ZERO(&w);
		FD_SET(c1, &w);
		FD_SET(c2, &w);
		FD_SET(p1, &w);
		FD_SET(p2, &w);

		s = select(max_fd + 1, &w, NULL, NULL, &timeout);
		if (s == -1) {
			perror("select()");
			break;
		} else if (!s) {
			fprintf(stderr, "unexpected timeout\n");
			break;
		}

		for (i = 0; i <= max_fd && s > 0; ++i) {
			if (!FD_ISSET(i, &w))
				continue;

			s--;

			rc = recv(i, buf, sizeof(buf), 0);
			if (rc < 0) {
				if (errno != EWOULDBLOCK) {
					perror("recv failed()\n");
					return rc;
				}
			}

			if (rc == 0) {
				close(i);
				break;
			}

			sc = send(i, buf, rc, 0);
			if (sc < 0) {
				perror("send failed()\n");
				return sc;
			}
		}

		if (rate)
			sleep(rate);

		if (verbose) {
			printf(".");
			fflush(stdout);

		}
	} while (running);

	return 0;
}

enum {
	PING_PONG,
	SENDMSG,
	BASE,
};

int main(int argc, char **argv)
{
	int iov_count = 1, length = 1024, rate = 1, verbose = 0;
	struct rlimit r = {10 * 1024 * 1024, RLIM_INFINITY};
	int opt, longindex, err, cg_fd = 0;
	int test = PING_PONG;
	char filename[256];

	while ((opt = getopt_long(argc, argv, "hvc:r:i:l:t:",
				  long_options, &longindex)) != -1) {
		switch (opt) {
		/* Cgroup configuration */
		case 'c':
			cg_fd = open(optarg, O_DIRECTORY, O_RDONLY);
			if (cg_fd < 0) {
				fprintf(stderr,
					"ERROR: (%i) open cg path failed: %s\n",
					cg_fd, optarg);
				return cg_fd;
			}
			break;
		case 'r':
			rate = atoi(optarg);
			break;
		case 'v':
			verbose = 1;
			break;
		case 'i':
			iov_count = atoi(optarg);
			break;
		case 'l':
			length = atoi(optarg);
			break;
		case 't':
			if (strcmp(optarg, "ping") == 0) {
				test = PING_PONG;
			} else if (strcmp(optarg, "sendmsg") == 0) {
				test = SENDMSG;
			} else if (strcmp(optarg, "base") == 0) {
				test = BASE;
			} else {
				usage(argv);
				return -1;
			}
			break;
		case 'h':
		default:
			usage(argv);
			return -1;
		}
	}

	if (!cg_fd) {
		fprintf(stderr, "%s requires cgroup option: --cgroup <path>\n",
			argv[0]);
		return -1;
	}

	if (setrlimit(RLIMIT_MEMLOCK, &r)) {
		perror("setrlimit(RLIMIT_MEMLOCK)");
		return 1;
	}

	snprintf(filename, sizeof(filename), "%s_kern.o", argv[0]);

	running = 1;

	/* catch SIGINT */
	signal(SIGINT, running_handler);

	/* If base test skip BPF setup */
	if (test == BASE)
		goto run;

	if (load_bpf_file(filename)) {
		fprintf(stderr, "load_bpf_file: (%s) %s\n",
			filename, strerror(errno));
		return 1;
	}

	/* Attach programs to sockmap */
	err = bpf_prog_attach(prog_fd[0], map_fd[0],
				BPF_SK_SKB_STREAM_PARSER, 0);
	if (err) {
		fprintf(stderr, "ERROR: bpf_prog_attach (sockmap): %d (%s)\n",
			err, strerror(errno));
		return err;
	}

	err = bpf_prog_attach(prog_fd[1], map_fd[0],
				BPF_SK_SKB_STREAM_VERDICT, 0);
	if (err) {
		fprintf(stderr, "ERROR: bpf_prog_attach (sockmap): %d (%s)\n",
			err, strerror(errno));
		return err;
	}

	/* Attach to cgroups */
	err = bpf_prog_attach(prog_fd[2], cg_fd, BPF_CGROUP_SOCK_OPS, 0);
	if (err) {
		fprintf(stderr, "ERROR: bpf_prog_attach (groups): %d (%s)\n",
			err, strerror(errno));
		return err;
	}

run:
	err = sockmap_init_sockets();
	if (err) {
		fprintf(stderr, "ERROR: test socket failed: %d\n", err);
		goto out;
	}

	if (test == PING_PONG)
		err = forever_ping_pong(rate, verbose);
	else if (test == SENDMSG)
		err = sendmsg_test(iov_count, length, rate, verbose, false);
	else if (test == BASE)
		err = sendmsg_test(iov_count, length, rate, verbose, true);
	else
		fprintf(stderr, "unknown test\n");
out:
	close(s1);
	close(s2);
	close(p1);
	close(p2);
	close(c1);
	close(c2);
	close(cg_fd);
	return err;
}

void running_handler(int a)
{
	running = 0;
}
