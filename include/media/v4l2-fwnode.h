/*
 * V4L2 fwnode binding parsing library
 *
 * Copyright (c) 2016 Intel Corporation.
 * Author: Sakari Ailus <sakari.ailus@linux.intel.com>
 *
 * Copyright (C) 2012 - 2013 Samsung Electronics Co., Ltd.
 * Author: Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Copyright (C) 2012 Renesas Electronics Corp.
 * Author: Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 */
#ifndef _V4L2_FWNODE_H
#define _V4L2_FWNODE_H

#include <linux/errno.h>
#include <linux/fwnode.h>
#include <linux/list.h>
#include <linux/types.h>

#include <media/v4l2-mediabus.h>

struct fwnode_handle;
struct v4l2_async_notifier;
struct v4l2_async_subdev;

#define V4L2_FWNODE_CSI2_MAX_DATA_LANES	4

/**
 * struct v4l2_fwnode_bus_mipi_csi2 - MIPI CSI-2 bus data structure
 * @flags: media bus (V4L2_MBUS_*) flags
 * @data_lanes: an array of physical data lane indexes
 * @clock_lane: physical lane index of the clock lane
 * @num_data_lanes: number of data lanes
 * @lane_polarities: polarity of the lanes. The order is the same of
 *		   the physical lanes.
 */
struct v4l2_fwnode_bus_mipi_csi2 {
	unsigned int flags;
	unsigned char data_lanes[V4L2_FWNODE_CSI2_MAX_DATA_LANES];
	unsigned char clock_lane;
	unsigned short num_data_lanes;
	bool lane_polarities[1 + V4L2_FWNODE_CSI2_MAX_DATA_LANES];
};

/**
 * struct v4l2_fwnode_bus_parallel - parallel data bus data structure
 * @flags: media bus (V4L2_MBUS_*) flags
 * @bus_width: bus width in bits
 * @data_shift: data shift in bits
 */
struct v4l2_fwnode_bus_parallel {
	unsigned int flags;
	unsigned char bus_width;
	unsigned char data_shift;
};

/**
 * struct v4l2_fwnode_bus_mipi_csi1 - CSI-1/CCP2 data bus structure
 * @clock_inv: polarity of clock/strobe signal
 *	       false - not inverted, true - inverted
 * @strobe: false - data/clock, true - data/strobe
 * @lane_polarity: the polarities of the clock (index 0) and data lanes
 *		   index (1)
 * @data_lane: the number of the data lane
 * @clock_lane: the number of the clock lane
 */
struct v4l2_fwnode_bus_mipi_csi1 {
	bool clock_inv;
	bool strobe;
	bool lane_polarity[2];
	unsigned char data_lane;
	unsigned char clock_lane;
};

/**
 * struct v4l2_fwnode_endpoint - the endpoint data structure
 * @base: fwnode endpoint of the v4l2_fwnode
 * @bus_type: bus type
 * @bus: bus configuration data structure
 * @link_frequencies: array of supported link frequencies
 * @nr_of_link_frequencies: number of elements in link_frequenccies array
 */
struct v4l2_fwnode_endpoint {
	struct fwnode_endpoint base;
	/*
	 * Fields below this line will be zeroed by
	 * v4l2_fwnode_parse_endpoint()
	 */
	enum v4l2_mbus_type bus_type;
	union {
		struct v4l2_fwnode_bus_parallel parallel;
		struct v4l2_fwnode_bus_mipi_csi1 mipi_csi1;
		struct v4l2_fwnode_bus_mipi_csi2 mipi_csi2;
	} bus;
	u64 *link_frequencies;
	unsigned int nr_of_link_frequencies;
};

/**
 * struct v4l2_fwnode_link - a link between two endpoints
 * @local_node: pointer to device_node of this endpoint
 * @local_port: identifier of the port this endpoint belongs to
 * @remote_node: pointer to device_node of the remote endpoint
 * @remote_port: identifier of the port the remote endpoint belongs to
 */
struct v4l2_fwnode_link {
	struct fwnode_handle *local_node;
	unsigned int local_port;
	struct fwnode_handle *remote_node;
	unsigned int remote_port;
};

/**
 * v4l2_fwnode_endpoint_parse() - parse all fwnode node properties
 * @fwnode: pointer to the endpoint's fwnode handle
 * @vep: pointer to the V4L2 fwnode data structure
 *
 * All properties are optional. If none are found, we don't set any flags. This
 * means the port has a static configuration and no properties have to be
 * specified explicitly. If any properties that identify the bus as parallel
 * are found and slave-mode isn't set, we set V4L2_MBUS_MASTER. Similarly, if
 * we recognise the bus as serial CSI-2 and clock-noncontinuous isn't set, we
 * set the V4L2_MBUS_CSI2_CONTINUOUS_CLOCK flag. The caller should hold a
 * reference to @fwnode.
 *
 * NOTE: This function does not parse properties the size of which is variable
 * without a low fixed limit. Please use v4l2_fwnode_endpoint_alloc_parse() in
 * new drivers instead.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int v4l2_fwnode_endpoint_parse(struct fwnode_handle *fwnode,
			       struct v4l2_fwnode_endpoint *vep);

/**
 * v4l2_fwnode_endpoint_free() - free the V4L2 fwnode acquired by
 * v4l2_fwnode_endpoint_alloc_parse()
 * @vep: the V4L2 fwnode the resources of which are to be released
 *
 * It is safe to call this function with NULL argument or on a V4L2 fwnode the
 * parsing of which failed.
 */
void v4l2_fwnode_endpoint_free(struct v4l2_fwnode_endpoint *vep);

/**
 * v4l2_fwnode_endpoint_alloc_parse() - parse all fwnode node properties
 * @fwnode: pointer to the endpoint's fwnode handle
 *
 * All properties are optional. If none are found, we don't set any flags. This
 * means the port has a static configuration and no properties have to be
 * specified explicitly. If any properties that identify the bus as parallel
 * are found and slave-mode isn't set, we set V4L2_MBUS_MASTER. Similarly, if
 * we recognise the bus as serial CSI-2 and clock-noncontinuous isn't set, we
 * set the V4L2_MBUS_CSI2_CONTINUOUS_CLOCK flag. The caller should hold a
 * reference to @fwnode.
 *
 * v4l2_fwnode_endpoint_alloc_parse() has two important differences to
 * v4l2_fwnode_endpoint_parse():
 *
 * 1. It also parses variable size data.
 *
 * 2. The memory it has allocated to store the variable size data must be freed
 *    using v4l2_fwnode_endpoint_free() when no longer needed.
 *
 * Return: Pointer to v4l2_fwnode_endpoint if successful, on an error pointer
 * on error.
 */
struct v4l2_fwnode_endpoint *v4l2_fwnode_endpoint_alloc_parse(
	struct fwnode_handle *fwnode);

/**
 * v4l2_fwnode_parse_link() - parse a link between two endpoints
 * @fwnode: pointer to the endpoint's fwnode at the local end of the link
 * @link: pointer to the V4L2 fwnode link data structure
 *
 * Fill the link structure with the local and remote nodes and port numbers.
 * The local_node and remote_node fields are set to point to the local and
 * remote port's parent nodes respectively (the port parent node being the
 * parent node of the port node if that node isn't a 'ports' node, or the
 * grand-parent node of the port node otherwise).
 *
 * A reference is taken to both the local and remote nodes, the caller must use
 * v4l2_fwnode_put_link() to drop the references when done with the
 * link.
 *
 * Return: 0 on success, or -ENOLINK if the remote endpoint fwnode can't be
 * found.
 */
int v4l2_fwnode_parse_link(struct fwnode_handle *fwnode,
			   struct v4l2_fwnode_link *link);

/**
 * v4l2_fwnode_put_link() - drop references to nodes in a link
 * @link: pointer to the V4L2 fwnode link data structure
 *
 * Drop references to the local and remote nodes in the link. This function
 * must be called on every link parsed with v4l2_fwnode_parse_link().
 */
void v4l2_fwnode_put_link(struct v4l2_fwnode_link *link);


/**
 * typedef parse_endpoint_func - Driver's callback function to be called on
 *	each V4L2 fwnode endpoint.
 *
 * @dev: pointer to &struct device
 * @vep: pointer to &struct v4l2_fwnode_endpoint
 * @asd: pointer to &struct v4l2_async_subdev
 *
 * Return:
 * * %0 on success
 * * %-ENOTCONN if the endpoint is to be skipped but this
 *   should not be considered as an error
 * * %-EINVAL if the endpoint configuration is invalid
 */
typedef int (*parse_endpoint_func)(struct device *dev,
				  struct v4l2_fwnode_endpoint *vep,
				  struct v4l2_async_subdev *asd);


/**
 * v4l2_async_notifier_parse_fwnode_endpoints - Parse V4L2 fwnode endpoints in a
 *						device node
 * @dev: the device the endpoints of which are to be parsed
 * @notifier: notifier for @dev
 * @asd_struct_size: size of the driver's async sub-device struct, including
 *		     sizeof(struct v4l2_async_subdev). The &struct
 *		     v4l2_async_subdev shall be the first member of
 *		     the driver's async sub-device struct, i.e. both
 *		     begin at the same memory address.
 * @parse_endpoint: Driver's callback function called on each V4L2 fwnode
 *		    endpoint. Optional.
 *
 * Parse the fwnode endpoints of the @dev device and populate the async sub-
 * devices array of the notifier. The @parse_endpoint callback function is
 * called for each endpoint with the corresponding async sub-device pointer to
 * let the caller initialize the driver-specific part of the async sub-device
 * structure.
 *
 * The notifier memory shall be zeroed before this function is called on the
 * notifier.
 *
 * This function may not be called on a registered notifier and may be called on
 * a notifier only once.
 *
 * Do not change the notifier's subdevs array, take references to the subdevs
 * array itself or change the notifier's num_subdevs field. This is because this
 * function allocates and reallocates the subdevs array based on parsing
 * endpoints.
 *
 * The &struct v4l2_fwnode_endpoint passed to the callback function
 * @parse_endpoint is released once the function is finished. If there is a need
 * to retain that configuration, the user needs to allocate memory for it.
 *
 * Any notifier populated using this function must be released with a call to
 * v4l2_async_notifier_cleanup() after it has been unregistered and the async
 * sub-devices are no longer in use, even if the function returned an error.
 *
 * Return: %0 on success, including when no async sub-devices are found
 *	   %-ENOMEM if memory allocation failed
 *	   %-EINVAL if graph or endpoint parsing failed
 *	   Other error codes as returned by @parse_endpoint
 */
int v4l2_async_notifier_parse_fwnode_endpoints(
	struct device *dev, struct v4l2_async_notifier *notifier,
	size_t asd_struct_size,
	parse_endpoint_func parse_endpoint);

/**
 * v4l2_async_notifier_parse_fwnode_endpoints_by_port - Parse V4L2 fwnode
 *							endpoints of a port in a
 *							device node
 * @dev: the device the endpoints of which are to be parsed
 * @notifier: notifier for @dev
 * @asd_struct_size: size of the driver's async sub-device struct, including
 *		     sizeof(struct v4l2_async_subdev). The &struct
 *		     v4l2_async_subdev shall be the first member of
 *		     the driver's async sub-device struct, i.e. both
 *		     begin at the same memory address.
 * @port: port number where endpoints are to be parsed
 * @parse_endpoint: Driver's callback function called on each V4L2 fwnode
 *		    endpoint. Optional.
 *
 * This function is just like v4l2_async_notifier_parse_fwnode_endpoints() with
 * the exception that it only parses endpoints in a given port. This is useful
 * on devices that have both sinks and sources: the async sub-devices connected
 * to sources have already been configured by another driver (on capture
 * devices). In this case the driver must know which ports to parse.
 *
 * Parse the fwnode endpoints of the @dev device on a given @port and populate
 * the async sub-devices array of the notifier. The @parse_endpoint callback
 * function is called for each endpoint with the corresponding async sub-device
 * pointer to let the caller initialize the driver-specific part of the async
 * sub-device structure.
 *
 * The notifier memory shall be zeroed before this function is called on the
 * notifier the first time.
 *
 * This function may not be called on a registered notifier and may be called on
 * a notifier only once per port.
 *
 * Do not change the notifier's subdevs array, take references to the subdevs
 * array itself or change the notifier's num_subdevs field. This is because this
 * function allocates and reallocates the subdevs array based on parsing
 * endpoints.
 *
 * The &struct v4l2_fwnode_endpoint passed to the callback function
 * @parse_endpoint is released once the function is finished. If there is a need
 * to retain that configuration, the user needs to allocate memory for it.
 *
 * Any notifier populated using this function must be released with a call to
 * v4l2_async_notifier_cleanup() after it has been unregistered and the async
 * sub-devices are no longer in use, even if the function returned an error.
 *
 * Return: %0 on success, including when no async sub-devices are found
 *	   %-ENOMEM if memory allocation failed
 *	   %-EINVAL if graph or endpoint parsing failed
 *	   Other error codes as returned by @parse_endpoint
 */
int v4l2_async_notifier_parse_fwnode_endpoints_by_port(
	struct device *dev, struct v4l2_async_notifier *notifier,
	size_t asd_struct_size, unsigned int port,
	parse_endpoint_func parse_endpoint);

/**
 * v4l2_fwnode_reference_parse_sensor_common - parse common references on
 *					       sensors for async sub-devices
 * @dev: the device node the properties of which are parsed for references
 * @notifier: the async notifier where the async subdevs will be added
 *
 * Parse common sensor properties for remote devices related to the
 * sensor and set up async sub-devices for them.
 *
 * Any notifier populated using this function must be released with a call to
 * v4l2_async_notifier_release() after it has been unregistered and the async
 * sub-devices are no longer in use, even in the case the function returned an
 * error.
 *
 * Return: 0 on success
 *	   -ENOMEM if memory allocation failed
 *	   -EINVAL if property parsing failed
 */
int v4l2_async_notifier_parse_fwnode_sensor_common(
	struct device *dev, struct v4l2_async_notifier *notifier);

#endif /* _V4L2_FWNODE_H */
