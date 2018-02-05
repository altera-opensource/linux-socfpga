// SPDX-License-Identifier: GPL-2.0
#define _GNU_SOURCE
#include <sched.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/limits.h>
#include <stdio.h>
#include <linux/sched.h>
#include <fcntl.h>
#include <unistd.h>
#include <ftw.h>


#include "cgroup_helpers.h"

/*
 * To avoid relying on the system setup, when setup_cgroup_env is called
 * we create a new mount namespace, and cgroup namespace. The cgroup2
 * root is mounted at CGROUP_MOUNT_PATH
 *
 * Unfortunately, most people don't have cgroupv2 enabled at this point in time.
 * It's easier to create our own mount namespace and manage it ourselves.
 *
 * We assume /mnt exists.
 */

#define WALK_FD_LIMIT			16
#define CGROUP_MOUNT_PATH		"/mnt"
#define CGROUP_WORK_DIR			"/cgroup-test-work-dir"
#define format_cgroup_path(buf, path) \
	snprintf(buf, sizeof(buf), "%s%s%s", CGROUP_MOUNT_PATH, \
		 CGROUP_WORK_DIR, path)

/**
 * setup_cgroup_environment() - Setup the cgroup environment
 *
 * After calling this function, cleanup_cgroup_environment should be called
 * once testing is complete.
 *
 * This function will print an error to stderr and return 1 if it is unable
 * to setup the cgroup environment. If setup is successful, 0 is returned.
 */
int setup_cgroup_environment(void)
{
	char cgroup_workdir[PATH_MAX + 1];

	format_cgroup_path(cgroup_workdir, "");

	if (unshare(CLONE_NEWNS)) {
		log_err("unshare");
		return 1;
	}

	if (mount("none", "/", NULL, MS_REC | MS_PRIVATE, NULL)) {
		log_err("mount fakeroot");
		return 1;
	}

	if (mount("none", CGROUP_MOUNT_PATH, "cgroup2", 0, NULL) && errno != EBUSY) {
		log_err("mount cgroup2");
		return 1;
	}

	/* Cleanup existing failed runs, now that the environment is setup */
	cleanup_cgroup_environment();

	if (mkdir(cgroup_workdir, 0777) && errno != EEXIST) {
		log_err("mkdir cgroup work dir");
		return 1;
	}

	return 0;
}

static int nftwfunc(const char *filename, const struct stat *statptr,
		    int fileflags, struct FTW *pfwt)
{
	if ((fileflags & FTW_D) && rmdir(filename))
		log_err("Removing cgroup: %s", filename);
	return 0;
}


static int join_cgroup_from_top(char *cgroup_path)
{
	char cgroup_procs_path[PATH_MAX + 1];
	pid_t pid = getpid();
	int fd, rc = 0;

	snprintf(cgroup_procs_path, sizeof(cgroup_procs_path),
		 "%s/cgroup.procs", cgroup_path);

	fd = open(cgroup_procs_path, O_WRONLY);
	if (fd < 0) {
		log_err("Opening Cgroup Procs: %s", cgroup_procs_path);
		return 1;
	}

	if (dprintf(fd, "%d\n", pid) < 0) {
		log_err("Joining Cgroup");
		rc = 1;
	}

	close(fd);
	return rc;
}

/**
 * join_cgroup() - Join a cgroup
 * @path: The cgroup path, relative to the workdir, to join
 *
 * This function expects a cgroup to already be created, relative to the cgroup
 * work dir, and it joins it. For example, passing "/my-cgroup" as the path
 * would actually put the calling process into the cgroup
 * "/cgroup-test-work-dir/my-cgroup"
 *
 * On success, it returns 0, otherwise on failure it returns 1.
 */
int join_cgroup(char *path)
{
	char cgroup_path[PATH_MAX + 1];

	format_cgroup_path(cgroup_path, path);
	return join_cgroup_from_top(cgroup_path);
}

/**
 * cleanup_cgroup_environment() - Cleanup Cgroup Testing Environment
 *
 * This is an idempotent function to delete all temporary cgroups that
 * have been created during the test, including the cgroup testing work
 * directory.
 *
 * At call time, it moves the calling process to the root cgroup, and then
 * runs the deletion process. It is idempotent, and should not fail, unless
 * a process is lingering.
 *
 * On failure, it will print an error to stderr, and try to continue.
 */
void cleanup_cgroup_environment(void)
{
	char cgroup_workdir[PATH_MAX + 1];

	format_cgroup_path(cgroup_workdir, "");
	join_cgroup_from_top(CGROUP_MOUNT_PATH);
	nftw(cgroup_workdir, nftwfunc, WALK_FD_LIMIT, FTW_DEPTH | FTW_MOUNT);
}

/**
 * create_and_get_cgroup() - Create a cgroup, relative to workdir, and get the FD
 * @path: The cgroup path, relative to the workdir, to join
 *
 * This function creates a cgroup under the top level workdir and returns the
 * file descriptor. It is idempotent.
 *
 * On success, it returns the file descriptor. On failure it returns 0.
 * If there is a failure, it prints the error to stderr.
 */
int create_and_get_cgroup(char *path)
{
	char cgroup_path[PATH_MAX + 1];
	int fd;

	format_cgroup_path(cgroup_path, path);
	if (mkdir(cgroup_path, 0777) && errno != EEXIST) {
		log_err("mkdiring cgroup %s .. %s", path, cgroup_path);
		return 0;
	}

	fd = open(cgroup_path, O_RDONLY);
	if (fd < 0) {
		log_err("Opening Cgroup");
		return 0;
	}

	return fd;
}
