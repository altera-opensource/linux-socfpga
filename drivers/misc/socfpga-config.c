// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025, Altera
 */

#include <linux/of_platform.h>
#include <linux/sysfs.h>
#include <misc/socfpga_fcs_hal.h>
#include <linux/platform_device.h>
#include <linux/of.h>

// Define the store function for the session_id attribute
static ssize_t open_session_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	ret = copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context));
	if (ret) {
		pr_err("Failed to copy context from user space ret: %d\n", ret);
		ret = -EFAULT;
		goto out;
	}

	ret = hal_session_open(k_ctx);
	if (ret)
		pr_err("Failed to open session\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

// Define the store function for the session_id attribute
static ssize_t close_session_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_session_close(k_ctx);
	if (ret)
		pr_err("Failed to close session\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t import_key_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_import_key(k_ctx);
	if (ret)
		pr_err("Failed to import key\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t export_key_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_export_key(k_ctx);
	if (ret)
		pr_err("Failed to export key\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t remove_key_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_remove_key(k_ctx);
	if (ret)
		pr_err("Failed to remove key\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t key_info_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_get_key_info(k_ctx);
	if (ret)
		pr_err("Failed to get key info\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t create_key_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_create_key(k_ctx);
	if (ret)
		pr_err("Failed to create key\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t hkdf_req_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_hkdf_request(k_ctx);
	if (ret)
		pr_err("Failed to hkdf request\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t prov_data_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_get_provision_data(k_ctx);
	if (ret)
		pr_err("Failed to get provision data\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ctr_set_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_counter_set(k_ctx);
	if (ret)
		pr_err("Failed to set counter\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ctr_set_preauth_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_counter_set_preauth(k_ctx);
	if (ret)
		pr_err("Failed to set counter preauth\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t context_info_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
	}

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t mac_verify_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_mac_verify(k_ctx);
	if (ret)
		pr_err("Failed to mac verify\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t aes_crypt_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_aes_crypt(k_ctx);
	if (ret)
		pr_err("Failed to crypt data\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ecdh_req_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_ecdh_req(k_ctx);
	if (ret)
		pr_err("Failed to perform ecdh request\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t chip_id_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_get_chip_id(k_ctx);
	if (ret)
		pr_err("Failed to get chip id\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t atstn_cert_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_attestation_get_certificate(k_ctx);
	if (ret)
		pr_err("Failed to get attestation cert\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t atstn_cert_reload_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_attestation_certificate_reload(k_ctx);
	if (ret)
		pr_err("Failed to get attestation cert\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t mctp_req_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_mctp_request(k_ctx);
	if (ret)
		pr_err("Failed to send mctp request\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t qspi_open_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_qspi_open(k_ctx);
	if (ret)
		pr_err("Failed to open qspi\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t qspi_close_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_qspi_close(k_ctx);
	if (ret)
		pr_err("Failed to close qspi\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t qspi_cs_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_qspi_cs(k_ctx);
	if (ret)
		pr_err("Failed to set qspi cs\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t qspi_read_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_qspi_read(k_ctx);
	if (ret)
		pr_err("Failed to read qspi\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t qspi_write_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_qspi_write(k_ctx);
	if (ret)
		pr_err("Failed to write qspi\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t qspi_erase_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_qspi_erase(k_ctx);
	if (ret)
		pr_err("Failed to erase qspi\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t jtag_idcode_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_jtag_idcode(k_ctx);
	if (ret)
		pr_err("Failed to get jtag idcode\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t device_identity_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_get_device_identity(k_ctx);
	if (ret)
		pr_err("Failed to get device identity\n");

out:

	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ecdsa_get_pubkey_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_ecdsa_get_pubkey(k_ctx);
	if (ret)
		pr_err("Failed to get ecdsa public key\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ecsda_hash_sign_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_ecdsa_hash_sign(k_ctx);
	if (ret)
		pr_err("Failed to sign hash\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ecdsa_hash_verify_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_ecdsa_hash_verify(k_ctx);
	if (ret)
		pr_err("Failed to verify hash\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ecdsa_sha2_data_sign_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_ecdsa_sha2_data_sign(k_ctx);
	if (ret)
		pr_err("Failed to sign data\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t ecdsa_sha2data_verify_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_ecdsa_sha2_data_verify(k_ctx);
	if (ret)
		pr_err("Failed to verify data\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t hps_image_validate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_hps_img_validate(k_ctx);
	if (ret)
		pr_err("Failed to validate HPS image\n");

out:
	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static ssize_t atf_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int version[3];

	hal_get_atf_version(version);
	return sprintf(buf, "%u.%u.%u\n", version[0], version[1], version[2]);
}

static ssize_t sdos_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t buf_size)
{
	struct fcs_cmd_context *const u_ctx = *(struct fcs_cmd_context **)buf;
	struct fcs_cmd_context *k_ctx;
	int ret = buf_size;

	k_ctx = hal_get_fcs_cmd_ctx();
	if (!k_ctx) {
		pr_err("Failed get context. Context is in use\n");
		ret = -EFAULT;
		goto out;
	}

	if (copy_from_user(k_ctx, u_ctx, sizeof(struct fcs_cmd_context))) {
		pr_err("Failed to copy context from user space\n");
		ret = -EFAULT;
		goto out;
	}

	ret = hal_sdos_crypt(k_ctx);
	if (ret)
		pr_err("Failed to get device identity\n");

out:

	hal_release_fcs_cmd_ctx(k_ctx);

	return ret;
}

static DEVICE_ATTR_WO(open_session);
static DEVICE_ATTR_WO(close_session);
static DEVICE_ATTR_WO(context_info);
static DEVICE_ATTR_WO(import_key);
static DEVICE_ATTR_WO(export_key);
static DEVICE_ATTR_WO(remove_key);
static DEVICE_ATTR_WO(key_info);
static DEVICE_ATTR_WO(create_key);
static DEVICE_ATTR_WO(hkdf_req);
static DEVICE_ATTR_WO(prov_data);
static DEVICE_ATTR_WO(ctr_set);
static DEVICE_ATTR_WO(ctr_set_preauth);
static DEVICE_ATTR_WO(mac_verify);
static DEVICE_ATTR_WO(aes_crypt);
static DEVICE_ATTR_WO(ecdh_req);
static DEVICE_ATTR_WO(chip_id);
static DEVICE_ATTR_WO(atstn_cert);
static DEVICE_ATTR_WO(atstn_cert_reload);
static DEVICE_ATTR_WO(mctp_req);
static DEVICE_ATTR_WO(jtag_idcode);
static DEVICE_ATTR_WO(device_identity);
static DEVICE_ATTR_WO(qspi_open);
static DEVICE_ATTR_WO(qspi_close);
static DEVICE_ATTR_WO(qspi_cs);
static DEVICE_ATTR_WO(qspi_read);
static DEVICE_ATTR_WO(qspi_write);
static DEVICE_ATTR_WO(qspi_erase);
static DEVICE_ATTR_WO(ecdsa_get_pubkey);
static DEVICE_ATTR_WO(ecsda_hash_sign);
static DEVICE_ATTR_WO(ecdsa_hash_verify);
static DEVICE_ATTR_WO(ecdsa_sha2_data_sign);
static DEVICE_ATTR_WO(ecdsa_sha2data_verify);
static DEVICE_ATTR_WO(hps_image_validate);
static DEVICE_ATTR_RO(atf_version);
static DEVICE_ATTR_WO(sdos);

static struct attribute *fcs_security_attrs[] = {
	&dev_attr_open_session.attr,
	&dev_attr_close_session.attr,
	&dev_attr_context_info.attr,
	&dev_attr_import_key.attr,
	&dev_attr_export_key.attr,
	&dev_attr_remove_key.attr,
	&dev_attr_key_info.attr,
	&dev_attr_create_key.attr,
	&dev_attr_hkdf_req.attr,
	&dev_attr_prov_data.attr,
	&dev_attr_ctr_set.attr,
	&dev_attr_ctr_set_preauth.attr,
	&dev_attr_mac_verify.attr,
	&dev_attr_aes_crypt.attr,
	&dev_attr_ecdh_req.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_atstn_cert.attr,
	&dev_attr_atstn_cert_reload.attr,
	&dev_attr_mctp_req.attr,
	&dev_attr_jtag_idcode.attr,
	&dev_attr_device_identity.attr,
	&dev_attr_qspi_open.attr,
	&dev_attr_qspi_close.attr,
	&dev_attr_qspi_cs.attr,
	&dev_attr_qspi_read.attr,
	&dev_attr_qspi_write.attr,
	&dev_attr_qspi_erase.attr,
	&dev_attr_ecdsa_get_pubkey.attr,
	&dev_attr_ecsda_hash_sign.attr,
	&dev_attr_ecdsa_hash_verify.attr,
	&dev_attr_ecdsa_sha2_data_sign.attr,
	&dev_attr_ecdsa_sha2data_verify.attr,
	&dev_attr_hps_image_validate.attr,
	&dev_attr_atf_version.attr,
	&dev_attr_sdos.attr,
	NULL
};

static struct attribute_group fcs_group = {
	.attrs = fcs_security_attrs,
};

static const struct attribute_group *fcs_groups[] = {
	&fcs_group,
	NULL,
};

struct kobject *sysfs_kobj;

static int fcs_driver_probe(struct platform_device *pdev)
{
	int ret;
	/* device node pointer */
	struct device_node *np = pdev->dev.of_node;
	struct device_node *fcs_hal_np;

	fcs_hal_np = of_parse_phandle(np, "dependent-on", 0);
	if (!fcs_hal_np) {
		pr_err("Failed to find HAL Driver device\n");
		return -ENODEV;
	}

	if (of_device_is_compatible(fcs_hal_np, "intel,agilex5-soc-fcs-hal") &&
	    !of_device_is_available(fcs_hal_np)) {
		pr_err("FCS HAL is not available.\n");
		of_node_put(fcs_hal_np);
		return -ENODEV;
	}

	sysfs_kobj = kobject_create_and_add("fcs_sysfs", kernel_kobj);
	if (!sysfs_kobj) {
		pr_err("Failed to create and add kobject\n");
		return -ENOMEM;
	}

	ret = sysfs_create_groups(sysfs_kobj, fcs_groups);
	if (ret)
		sysfs_remove_groups(sysfs_kobj, fcs_groups);

	pr_info("FCS Security Driver probed successfully\n");

	return ret;
}

static const struct of_device_id fcs_of_match[] = {
	{ .compatible = "intel,agilex5-soc-fcs-config" },
	{},
};

static struct platform_driver fcs_driver = {
	.probe = fcs_driver_probe,
	.driver = {
		.name = "socfpga-security",
		.of_match_table = of_match_ptr(fcs_of_match),
	},
};

MODULE_DEVICE_TABLE(of, fcs_of_match);

static int __init fcs_security_init(void)
{
	struct device_node *fw_np;
	struct device_node *np;
	int ret;

	fw_np = of_find_node_by_name(NULL, "svc");
	if (!fw_np)
		return -ENODEV;

	of_node_get(fw_np);
	np = of_find_matching_node(fw_np, fcs_of_match);
	if (!np) {
		of_node_put(fw_np);
		return -ENODEV;
	}

	of_node_put(np);
	ret = of_platform_populate(fw_np, fcs_of_match, NULL, NULL);
	of_node_put(fw_np);
	if (ret)
		return ret;

	ret = platform_driver_register(&fcs_driver);
	if (ret)
		pr_err("Failed to register platform driver\n");

	return ret;
}

static void __exit fcs_security_exit(void)
{
	/* Remove sysfs groups */
	sysfs_remove_groups(sysfs_kobj, fcs_groups);

	/* Remove the kobject */
	if (sysfs_kobj)
		kobject_put(sysfs_kobj);

	return platform_driver_unregister(&fcs_driver);
}

module_init(fcs_security_init);
module_exit(fcs_security_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Altera socfpga security driver");
MODULE_AUTHOR("Balsundar Ponnusamy, Santosh Male, Sagar Khadgi");
