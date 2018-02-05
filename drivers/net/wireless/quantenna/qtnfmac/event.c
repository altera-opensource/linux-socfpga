/*
 * Copyright (c) 2015-2016 Quantenna Communications, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "cfg80211.h"
#include "core.h"
#include "qlink.h"
#include "bus.h"
#include "trans.h"
#include "util.h"
#include "event.h"
#include "qlink_util.h"

static int
qtnf_event_handle_sta_assoc(struct qtnf_wmac *mac, struct qtnf_vif *vif,
			    const struct qlink_event_sta_assoc *sta_assoc,
			    u16 len)
{
	const u8 *sta_addr;
	u16 frame_control;
	struct station_info sinfo = { 0 };
	size_t payload_len;
	u16 tlv_type;
	u16 tlv_value_len;
	size_t tlv_full_len;
	const struct qlink_tlv_hdr *tlv;

	if (unlikely(len < sizeof(*sta_assoc))) {
		pr_err("VIF%u.%u: payload is too short (%u < %zu)\n",
		       mac->macid, vif->vifid, len, sizeof(*sta_assoc));
		return -EINVAL;
	}

	if (vif->wdev.iftype != NL80211_IFTYPE_AP) {
		pr_err("VIF%u.%u: STA_ASSOC event when not in AP mode\n",
		       mac->macid, vif->vifid);
		return -EPROTO;
	}

	sta_addr = sta_assoc->sta_addr;
	frame_control = le16_to_cpu(sta_assoc->frame_control);

	pr_debug("VIF%u.%u: MAC:%pM FC:%x\n", mac->macid, vif->vifid, sta_addr,
		 frame_control);

	qtnf_sta_list_add(vif, sta_addr);

	sinfo.assoc_req_ies = NULL;
	sinfo.assoc_req_ies_len = 0;
	sinfo.generation = vif->generation;

	payload_len = len - sizeof(*sta_assoc);
	tlv = (const struct qlink_tlv_hdr *)sta_assoc->ies;

	while (payload_len >= sizeof(*tlv)) {
		tlv_type = le16_to_cpu(tlv->type);
		tlv_value_len = le16_to_cpu(tlv->len);
		tlv_full_len = tlv_value_len + sizeof(struct qlink_tlv_hdr);

		if (tlv_full_len > payload_len)
			return -EINVAL;

		if (tlv_type == QTN_TLV_ID_IE_SET) {
			const struct qlink_tlv_ie_set *ie_set;
			unsigned int ie_len;

			if (payload_len < sizeof(*ie_set))
				return -EINVAL;

			ie_set = (const struct qlink_tlv_ie_set *)tlv;
			ie_len = tlv_value_len -
				(sizeof(*ie_set) - sizeof(ie_set->hdr));

			if (ie_set->type == QLINK_IE_SET_ASSOC_REQ && ie_len) {
				sinfo.assoc_req_ies = ie_set->ie_data;
				sinfo.assoc_req_ies_len = ie_len;
			}
		}

		payload_len -= tlv_full_len;
		tlv = (struct qlink_tlv_hdr *)(tlv->val + tlv_value_len);
	}

	if (payload_len)
		return -EINVAL;

	cfg80211_new_sta(vif->netdev, sta_assoc->sta_addr, &sinfo,
			 GFP_KERNEL);

	return 0;
}

static int
qtnf_event_handle_sta_deauth(struct qtnf_wmac *mac, struct qtnf_vif *vif,
			     const struct qlink_event_sta_deauth *sta_deauth,
			     u16 len)
{
	const u8 *sta_addr;
	u16 reason;

	if (unlikely(len < sizeof(*sta_deauth))) {
		pr_err("VIF%u.%u: payload is too short (%u < %zu)\n",
		       mac->macid, vif->vifid, len,
		       sizeof(struct qlink_event_sta_deauth));
		return -EINVAL;
	}

	if (vif->wdev.iftype != NL80211_IFTYPE_AP) {
		pr_err("VIF%u.%u: STA_DEAUTH event when not in AP mode\n",
		       mac->macid, vif->vifid);
		return -EPROTO;
	}

	sta_addr = sta_deauth->sta_addr;
	reason = le16_to_cpu(sta_deauth->reason);

	pr_debug("VIF%u.%u: MAC:%pM reason:%x\n", mac->macid, vif->vifid,
		 sta_addr, reason);

	if (qtnf_sta_list_del(vif, sta_addr))
		cfg80211_del_sta(vif->netdev, sta_deauth->sta_addr,
				 GFP_KERNEL);

	return 0;
}

static int
qtnf_event_handle_bss_join(struct qtnf_vif *vif,
			   const struct qlink_event_bss_join *join_info,
			   u16 len)
{
	if (unlikely(len < sizeof(*join_info))) {
		pr_err("VIF%u.%u: payload is too short (%u < %zu)\n",
		       vif->mac->macid, vif->vifid, len,
		       sizeof(struct qlink_event_bss_join));
		return -EINVAL;
	}

	if (vif->wdev.iftype != NL80211_IFTYPE_STATION) {
		pr_err("VIF%u.%u: BSS_JOIN event when not in STA mode\n",
		       vif->mac->macid, vif->vifid);
		return -EPROTO;
	}

	if (vif->sta_state != QTNF_STA_CONNECTING) {
		pr_err("VIF%u.%u: BSS_JOIN event when STA is not connecting\n",
		       vif->mac->macid, vif->vifid);
		return -EPROTO;
	}

	pr_debug("VIF%u.%u: BSSID:%pM\n", vif->mac->macid, vif->vifid,
		 join_info->bssid);

	cfg80211_connect_result(vif->netdev, join_info->bssid, NULL, 0, NULL,
				0, le16_to_cpu(join_info->status), GFP_KERNEL);

	if (le16_to_cpu(join_info->status) == WLAN_STATUS_SUCCESS) {
		vif->sta_state = QTNF_STA_CONNECTED;
		netif_carrier_on(vif->netdev);
	} else {
		vif->sta_state = QTNF_STA_DISCONNECTED;
	}

	return 0;
}

static int
qtnf_event_handle_bss_leave(struct qtnf_vif *vif,
			    const struct qlink_event_bss_leave *leave_info,
			    u16 len)
{
	if (unlikely(len < sizeof(*leave_info))) {
		pr_err("VIF%u.%u: payload is too short (%u < %zu)\n",
		       vif->mac->macid, vif->vifid, len,
		       sizeof(struct qlink_event_bss_leave));
		return -EINVAL;
	}

	if (vif->wdev.iftype != NL80211_IFTYPE_STATION) {
		pr_err("VIF%u.%u: BSS_LEAVE event when not in STA mode\n",
		       vif->mac->macid, vif->vifid);
		return -EPROTO;
	}

	if (vif->sta_state != QTNF_STA_CONNECTED) {
		pr_err("VIF%u.%u: BSS_LEAVE event when STA is not connected\n",
		       vif->mac->macid, vif->vifid);
		return -EPROTO;
	}

	pr_debug("VIF%u.%u: disconnected\n", vif->mac->macid, vif->vifid);

	cfg80211_disconnected(vif->netdev, le16_to_cpu(leave_info->reason),
			      NULL, 0, 0, GFP_KERNEL);

	vif->sta_state = QTNF_STA_DISCONNECTED;
	netif_carrier_off(vif->netdev);

	return 0;
}

static int
qtnf_event_handle_mgmt_received(struct qtnf_vif *vif,
				const struct qlink_event_rxmgmt *rxmgmt,
				u16 len)
{
	const size_t min_len = sizeof(*rxmgmt) +
			       sizeof(struct ieee80211_hdr_3addr);
	const struct ieee80211_hdr_3addr *frame = (void *)rxmgmt->frame_data;
	const u16 frame_len = len - sizeof(*rxmgmt);
	enum nl80211_rxmgmt_flags flags = 0;

	if (unlikely(len < min_len)) {
		pr_err("VIF%u.%u: payload is too short (%u < %zu)\n",
		       vif->mac->macid, vif->vifid, len, min_len);
		return -EINVAL;
	}

	if (le32_to_cpu(rxmgmt->flags) & QLINK_RXMGMT_FLAG_ANSWERED)
		flags |= NL80211_RXMGMT_FLAG_ANSWERED;

	pr_debug("%s LEN:%u FC:%.4X SA:%pM\n", vif->netdev->name, frame_len,
		 le16_to_cpu(frame->frame_control), frame->addr2);

	cfg80211_rx_mgmt(&vif->wdev, le32_to_cpu(rxmgmt->freq), rxmgmt->sig_dbm,
			 rxmgmt->frame_data, frame_len, flags);

	return 0;
}

static int
qtnf_event_handle_scan_results(struct qtnf_vif *vif,
			       const struct qlink_event_scan_result *sr,
			       u16 len)
{
	struct cfg80211_bss *bss;
	struct ieee80211_channel *channel;
	struct wiphy *wiphy = priv_to_wiphy(vif->mac);
	enum cfg80211_bss_frame_type frame_type = CFG80211_BSS_FTYPE_UNKNOWN;
	size_t payload_len;
	u16 tlv_type;
	u16 tlv_value_len;
	size_t tlv_full_len;
	const struct qlink_tlv_hdr *tlv;
	const u8 *ies = NULL;
	size_t ies_len = 0;

	if (len < sizeof(*sr)) {
		pr_err("VIF%u.%u: payload is too short\n", vif->mac->macid,
		       vif->vifid);
		return -EINVAL;
	}

	channel = ieee80211_get_channel(wiphy, le16_to_cpu(sr->freq));
	if (!channel) {
		pr_err("VIF%u.%u: channel at %u MHz not found\n",
		       vif->mac->macid, vif->vifid, le16_to_cpu(sr->freq));
		return -EINVAL;
	}

	payload_len = len - sizeof(*sr);
	tlv = (struct qlink_tlv_hdr *)sr->payload;

	while (payload_len >= sizeof(struct qlink_tlv_hdr)) {
		tlv_type = le16_to_cpu(tlv->type);
		tlv_value_len = le16_to_cpu(tlv->len);
		tlv_full_len = tlv_value_len + sizeof(struct qlink_tlv_hdr);

		if (tlv_full_len > payload_len)
			return -EINVAL;

		if (tlv_type == QTN_TLV_ID_IE_SET) {
			const struct qlink_tlv_ie_set *ie_set;
			unsigned int ie_len;

			if (payload_len < sizeof(*ie_set))
				return -EINVAL;

			ie_set = (const struct qlink_tlv_ie_set *)tlv;
			ie_len = tlv_value_len -
				(sizeof(*ie_set) - sizeof(ie_set->hdr));

			switch (ie_set->type) {
			case QLINK_IE_SET_BEACON_IES:
				frame_type = CFG80211_BSS_FTYPE_BEACON;
				break;
			case QLINK_IE_SET_PROBE_RESP_IES:
				frame_type = CFG80211_BSS_FTYPE_PRESP;
				break;
			default:
				frame_type = CFG80211_BSS_FTYPE_UNKNOWN;
			}

			if (ie_len) {
				ies = ie_set->ie_data;
				ies_len = ie_len;
			}
		}

		payload_len -= tlv_full_len;
		tlv = (struct qlink_tlv_hdr *)(tlv->val + tlv_value_len);
	}

	if (payload_len)
		return -EINVAL;

	bss = cfg80211_inform_bss(wiphy, channel, frame_type,
				  sr->bssid, get_unaligned_le64(&sr->tsf),
				  le16_to_cpu(sr->capab),
				  le16_to_cpu(sr->bintval), ies, ies_len,
				  DBM_TO_MBM(sr->sig_dbm), GFP_KERNEL);
	if (!bss)
		return -ENOMEM;

	cfg80211_put_bss(wiphy, bss);

	return 0;
}

static int
qtnf_event_handle_scan_complete(struct qtnf_wmac *mac,
				const struct qlink_event_scan_complete *status,
				u16 len)
{
	if (len < sizeof(*status)) {
		pr_err("MAC%u: payload is too short\n", mac->macid);
		return -EINVAL;
	}

	qtnf_scan_done(mac, le32_to_cpu(status->flags) & QLINK_SCAN_ABORTED);

	return 0;
}

static int
qtnf_event_handle_freq_change(struct qtnf_wmac *mac,
			      const struct qlink_event_freq_change *data,
			      u16 len)
{
	struct wiphy *wiphy = priv_to_wiphy(mac);
	struct cfg80211_chan_def chandef;
	struct qtnf_vif *vif;
	int i;

	if (len < sizeof(*data)) {
		pr_err("MAC%u: payload is too short\n", mac->macid);
		return -EINVAL;
	}

	if (!wiphy->registered)
		return 0;

	qlink_chandef_q2cfg(wiphy, &data->chan, &chandef);

	if (!cfg80211_chandef_valid(&chandef)) {
		pr_err("MAC%u: bad channel freq=%u cf1=%u cf2=%u bw=%u\n",
		       mac->macid, chandef.chan->center_freq,
		       chandef.center_freq1, chandef.center_freq2,
		       chandef.width);
		return -EINVAL;
	}

	pr_debug("MAC%d: new channel ieee=%u freq1=%u freq2=%u bw=%u\n",
		 mac->macid, chandef.chan->hw_value, chandef.center_freq1,
		 chandef.center_freq2, chandef.width);

	for (i = 0; i < QTNF_MAX_INTF; i++) {
		vif = &mac->iflist[i];
		if (vif->wdev.iftype == NL80211_IFTYPE_UNSPECIFIED)
			continue;

		if (vif->netdev) {
			mutex_lock(&vif->wdev.mtx);
			cfg80211_ch_switch_notify(vif->netdev, &chandef);
			mutex_unlock(&vif->wdev.mtx);
		}
	}

	return 0;
}

static int qtnf_event_handle_radar(struct qtnf_vif *vif,
				   const struct qlink_event_radar *ev,
				   u16 len)
{
	struct wiphy *wiphy = priv_to_wiphy(vif->mac);
	struct cfg80211_chan_def chandef;

	if (len < sizeof(*ev)) {
		pr_err("MAC%u: payload is too short\n", vif->mac->macid);
		return -EINVAL;
	}

	if (!wiphy->registered || !vif->netdev)
		return 0;

	qlink_chandef_q2cfg(wiphy, &ev->chan, &chandef);

	if (!cfg80211_chandef_valid(&chandef)) {
		pr_err("MAC%u: bad channel f1=%u f2=%u bw=%u\n",
		       vif->mac->macid,
		       chandef.center_freq1, chandef.center_freq2,
		       chandef.width);
		return -EINVAL;
	}

	pr_info("%s: radar event=%u f1=%u f2=%u bw=%u\n",
		vif->netdev->name, ev->event,
		chandef.center_freq1, chandef.center_freq2,
		chandef.width);

	switch (ev->event) {
	case QLINK_RADAR_DETECTED:
		cfg80211_radar_event(wiphy, &chandef, GFP_KERNEL);
		break;
	case QLINK_RADAR_CAC_FINISHED:
		if (!vif->wdev.cac_started)
			break;

		cfg80211_cac_event(vif->netdev, &chandef,
				   NL80211_RADAR_CAC_FINISHED, GFP_KERNEL);
		break;
	case QLINK_RADAR_CAC_ABORTED:
		if (!vif->wdev.cac_started)
			break;

		cfg80211_cac_event(vif->netdev, &chandef,
				   NL80211_RADAR_CAC_ABORTED, GFP_KERNEL);
		break;
	default:
		pr_warn("%s: unhandled radar event %u\n",
			vif->netdev->name, ev->event);
		break;
	}

	return 0;
}

static int qtnf_event_parse(struct qtnf_wmac *mac,
			    const struct sk_buff *event_skb)
{
	const struct qlink_event *event;
	struct qtnf_vif *vif = NULL;
	int ret = -1;
	u16 event_id;
	u16 event_len;

	event = (const struct qlink_event *)event_skb->data;
	event_id = le16_to_cpu(event->event_id);
	event_len = le16_to_cpu(event->mhdr.len);

	if (likely(event->vifid < QTNF_MAX_INTF)) {
		vif = &mac->iflist[event->vifid];
	} else {
		pr_err("invalid vif(%u)\n", event->vifid);
		return -EINVAL;
	}

	switch (event_id) {
	case QLINK_EVENT_STA_ASSOCIATED:
		ret = qtnf_event_handle_sta_assoc(mac, vif, (const void *)event,
						  event_len);
		break;
	case QLINK_EVENT_STA_DEAUTH:
		ret = qtnf_event_handle_sta_deauth(mac, vif,
						   (const void *)event,
						   event_len);
		break;
	case QLINK_EVENT_MGMT_RECEIVED:
		ret = qtnf_event_handle_mgmt_received(vif, (const void *)event,
						      event_len);
		break;
	case QLINK_EVENT_SCAN_RESULTS:
		ret = qtnf_event_handle_scan_results(vif, (const void *)event,
						     event_len);
		break;
	case QLINK_EVENT_SCAN_COMPLETE:
		ret = qtnf_event_handle_scan_complete(mac, (const void *)event,
						      event_len);
		break;
	case QLINK_EVENT_BSS_JOIN:
		ret = qtnf_event_handle_bss_join(vif, (const void *)event,
						 event_len);
		break;
	case QLINK_EVENT_BSS_LEAVE:
		ret = qtnf_event_handle_bss_leave(vif, (const void *)event,
						  event_len);
		break;
	case QLINK_EVENT_FREQ_CHANGE:
		ret = qtnf_event_handle_freq_change(mac, (const void *)event,
						    event_len);
		break;
	case QLINK_EVENT_RADAR:
		ret = qtnf_event_handle_radar(vif, (const void *)event,
					      event_len);
		break;
	default:
		pr_warn("unknown event type: %x\n", event_id);
		break;
	}

	return ret;
}

static int qtnf_event_process_skb(struct qtnf_bus *bus,
				  const struct sk_buff *skb)
{
	const struct qlink_event *event;
	struct qtnf_wmac *mac;
	int res;

	if (unlikely(!skb || skb->len < sizeof(*event))) {
		pr_err("invalid event buffer\n");
		return -EINVAL;
	}

	event = (struct qlink_event *)skb->data;

	mac = qtnf_core_get_mac(bus, event->macid);

	pr_debug("new event id:%x len:%u mac:%u vif:%u\n",
		 le16_to_cpu(event->event_id), le16_to_cpu(event->mhdr.len),
		 event->macid, event->vifid);

	if (unlikely(!mac))
		return -ENXIO;

	rtnl_lock();
	res = qtnf_event_parse(mac, skb);
	rtnl_unlock();

	return res;
}

void qtnf_event_work_handler(struct work_struct *work)
{
	struct qtnf_bus *bus = container_of(work, struct qtnf_bus, event_work);
	struct sk_buff_head *event_queue = &bus->trans.event_queue;
	struct sk_buff *current_event_skb = skb_dequeue(event_queue);

	while (current_event_skb) {
		qtnf_event_process_skb(bus, current_event_skb);
		dev_kfree_skb_any(current_event_skb);
		current_event_skb = skb_dequeue(event_queue);
	}
}
