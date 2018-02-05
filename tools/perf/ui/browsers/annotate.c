// SPDX-License-Identifier: GPL-2.0
#include "../../util/util.h"
#include "../browser.h"
#include "../helpline.h"
#include "../ui.h"
#include "../util.h"
#include "../../util/annotate.h"
#include "../../util/hist.h"
#include "../../util/sort.h"
#include "../../util/symbol.h"
#include "../../util/evsel.h"
#include "../../util/config.h"
#include "../../util/evlist.h"
#include <inttypes.h>
#include <pthread.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <sys/ttydefaults.h>

struct disasm_line_samples {
	double		      percent;
	struct sym_hist_entry he;
};

#define IPC_WIDTH 6
#define CYCLES_WIDTH 6

struct browser_line {
	u32	idx;
	int	idx_asm;
	int	jump_sources;
};

static struct annotate_browser_opt {
	bool hide_src_code,
	     use_offset,
	     jump_arrows,
	     show_linenr,
	     show_nr_jumps,
	     show_nr_samples,
	     show_total_period;
} annotate_browser__opts = {
	.use_offset	= true,
	.jump_arrows	= true,
};

struct arch;

struct annotate_browser {
	struct ui_browser	    b;
	struct rb_root		    entries;
	struct rb_node		   *curr_hot;
	struct annotation_line	   *selection;
	struct annotation_line	  **offsets;
	struct arch		   *arch;
	int			    nr_events;
	u64			    start;
	int			    nr_asm_entries;
	int			    nr_entries;
	int			    max_jump_sources;
	int			    nr_jumps;
	bool			    searching_backwards;
	bool			    have_cycles;
	u8			    addr_width;
	u8			    jumps_width;
	u8			    target_width;
	u8			    min_addr_width;
	u8			    max_addr_width;
	char			    search_bf[128];
};

static inline struct browser_line *browser_line(struct annotation_line *al)
{
	void *ptr = al;

	ptr = container_of(al, struct disasm_line, al);
	return ptr - sizeof(struct browser_line);
}

static bool disasm_line__filter(struct ui_browser *browser __maybe_unused,
				void *entry)
{
	if (annotate_browser__opts.hide_src_code) {
		struct annotation_line *al = list_entry(entry, struct annotation_line, node);

		return al->offset == -1;
	}

	return false;
}

static int annotate_browser__jumps_percent_color(struct annotate_browser *browser,
						 int nr, bool current)
{
	if (current && (!browser->b.use_navkeypressed || browser->b.navkeypressed))
		return HE_COLORSET_SELECTED;
	if (nr == browser->max_jump_sources)
		return HE_COLORSET_TOP;
	if (nr > 1)
		return HE_COLORSET_MEDIUM;
	return HE_COLORSET_NORMAL;
}

static int annotate_browser__set_jumps_percent_color(struct annotate_browser *browser,
						     int nr, bool current)
{
	 int color = annotate_browser__jumps_percent_color(browser, nr, current);
	 return ui_browser__set_color(&browser->b, color);
}

static int annotate_browser__pcnt_width(struct annotate_browser *ab)
{
	return (annotate_browser__opts.show_total_period ? 12 : 7) * ab->nr_events;
}

static int annotate_browser__cycles_width(struct annotate_browser *ab)
{
	return ab->have_cycles ? IPC_WIDTH + CYCLES_WIDTH : 0;
}

static void disasm_line__write(struct disasm_line *dl, struct ui_browser *browser,
			       char *bf, size_t size)
{
	if (dl->ins.ops && dl->ins.ops->scnprintf) {
		if (ins__is_jump(&dl->ins)) {
			bool fwd = dl->ops.target.offset > dl->al.offset;

			ui_browser__write_graph(browser, fwd ? SLSMG_DARROW_CHAR :
							    SLSMG_UARROW_CHAR);
			SLsmg_write_char(' ');
		} else if (ins__is_call(&dl->ins)) {
			ui_browser__write_graph(browser, SLSMG_RARROW_CHAR);
			SLsmg_write_char(' ');
		} else if (ins__is_ret(&dl->ins)) {
			ui_browser__write_graph(browser, SLSMG_LARROW_CHAR);
			SLsmg_write_char(' ');
		} else {
			ui_browser__write_nstring(browser, " ", 2);
		}
	} else {
		ui_browser__write_nstring(browser, " ", 2);
	}

	disasm_line__scnprintf(dl, bf, size, !annotate_browser__opts.use_offset);
}

static void annotate_browser__write(struct ui_browser *browser, void *entry, int row)
{
	struct annotate_browser *ab = container_of(browser, struct annotate_browser, b);
	struct annotation_line *al = list_entry(entry, struct annotation_line, node);
	struct browser_line *bl = browser_line(al);
	bool current_entry = ui_browser__is_current_entry(browser, row);
	bool change_color = (!annotate_browser__opts.hide_src_code &&
			     (!current_entry || (browser->use_navkeypressed &&
					         !browser->navkeypressed)));
	int width = browser->width, printed;
	int i, pcnt_width = annotate_browser__pcnt_width(ab),
	       cycles_width = annotate_browser__cycles_width(ab);
	double percent_max = 0.0;
	char bf[256];
	bool show_title = false;

	for (i = 0; i < ab->nr_events; i++) {
		if (al->samples[i].percent > percent_max)
			percent_max = al->samples[i].percent;
	}

	if ((row == 0) && (al->offset == -1 || percent_max == 0.0)) {
		if (ab->have_cycles) {
			if (al->ipc == 0.0 && al->cycles == 0)
				show_title = true;
		} else
			show_title = true;
	}

	if (al->offset != -1 && percent_max != 0.0) {
		for (i = 0; i < ab->nr_events; i++) {
			ui_browser__set_percent_color(browser,
						al->samples[i].percent,
						current_entry);
			if (annotate_browser__opts.show_total_period) {
				ui_browser__printf(browser, "%11" PRIu64 " ",
						   al->samples[i].he.period);
			} else if (annotate_browser__opts.show_nr_samples) {
				ui_browser__printf(browser, "%6" PRIu64 " ",
						   al->samples[i].he.nr_samples);
			} else {
				ui_browser__printf(browser, "%6.2f ",
						   al->samples[i].percent);
			}
		}
	} else {
		ui_browser__set_percent_color(browser, 0, current_entry);

		if (!show_title)
			ui_browser__write_nstring(browser, " ", pcnt_width);
		else {
			ui_browser__printf(browser, "%*s", pcnt_width,
					   annotate_browser__opts.show_total_period ? "Period" :
					   annotate_browser__opts.show_nr_samples ? "Samples" : "Percent");
		}
	}
	if (ab->have_cycles) {
		if (al->ipc)
			ui_browser__printf(browser, "%*.2f ", IPC_WIDTH - 1, al->ipc);
		else if (!show_title)
			ui_browser__write_nstring(browser, " ", IPC_WIDTH);
		else
			ui_browser__printf(browser, "%*s ", IPC_WIDTH - 1, "IPC");

		if (al->cycles)
			ui_browser__printf(browser, "%*" PRIu64 " ",
					   CYCLES_WIDTH - 1, al->cycles);
		else if (!show_title)
			ui_browser__write_nstring(browser, " ", CYCLES_WIDTH);
		else
			ui_browser__printf(browser, "%*s ", CYCLES_WIDTH - 1, "Cycle");
	}

	SLsmg_write_char(' ');

	/* The scroll bar isn't being used */
	if (!browser->navkeypressed)
		width += 1;

	if (!*al->line)
		ui_browser__write_nstring(browser, " ", width - pcnt_width - cycles_width);
	else if (al->offset == -1) {
		if (al->line_nr && annotate_browser__opts.show_linenr)
			printed = scnprintf(bf, sizeof(bf), "%-*d ",
					ab->addr_width + 1, al->line_nr);
		else
			printed = scnprintf(bf, sizeof(bf), "%*s  ",
				    ab->addr_width, " ");
		ui_browser__write_nstring(browser, bf, printed);
		ui_browser__write_nstring(browser, al->line, width - printed - pcnt_width - cycles_width + 1);
	} else {
		u64 addr = al->offset;
		int color = -1;

		if (!annotate_browser__opts.use_offset)
			addr += ab->start;

		if (!annotate_browser__opts.use_offset) {
			printed = scnprintf(bf, sizeof(bf), "%" PRIx64 ": ", addr);
		} else {
			if (bl->jump_sources) {
				if (annotate_browser__opts.show_nr_jumps) {
					int prev;
					printed = scnprintf(bf, sizeof(bf), "%*d ",
							    ab->jumps_width,
							    bl->jump_sources);
					prev = annotate_browser__set_jumps_percent_color(ab, bl->jump_sources,
											 current_entry);
					ui_browser__write_nstring(browser, bf, printed);
					ui_browser__set_color(browser, prev);
				}

				printed = scnprintf(bf, sizeof(bf), "%*" PRIx64 ": ",
						    ab->target_width, addr);
			} else {
				printed = scnprintf(bf, sizeof(bf), "%*s  ",
						    ab->addr_width, " ");
			}
		}

		if (change_color)
			color = ui_browser__set_color(browser, HE_COLORSET_ADDR);
		ui_browser__write_nstring(browser, bf, printed);
		if (change_color)
			ui_browser__set_color(browser, color);

		disasm_line__write(disasm_line(al), browser, bf, sizeof(bf));

		ui_browser__write_nstring(browser, bf, width - pcnt_width - cycles_width - 3 - printed);
	}

	if (current_entry)
		ab->selection = al;
}

static bool disasm_line__is_valid_jump(struct disasm_line *dl, struct symbol *sym)
{
	if (!dl || !dl->ins.ops || !ins__is_jump(&dl->ins)
	    || !disasm_line__has_offset(dl)
	    || dl->ops.target.offset < 0
	    || dl->ops.target.offset >= (s64)symbol__size(sym))
		return false;

	return true;
}

static bool is_fused(struct annotate_browser *ab, struct disasm_line *cursor)
{
	struct disasm_line *pos = list_prev_entry(cursor, al.node);
	const char *name;

	if (!pos)
		return false;

	if (ins__is_lock(&pos->ins))
		name = pos->ops.locked.ins.name;
	else
		name = pos->ins.name;

	if (!name || !cursor->ins.name)
		return false;

	return ins__is_fused(ab->arch, name, cursor->ins.name);
}

static void annotate_browser__draw_current_jump(struct ui_browser *browser)
{
	struct annotate_browser *ab = container_of(browser, struct annotate_browser, b);
	struct disasm_line *cursor = disasm_line(ab->selection);
	struct annotation_line *target;
	struct browser_line *btarget, *bcursor;
	unsigned int from, to;
	struct map_symbol *ms = ab->b.priv;
	struct symbol *sym = ms->sym;
	u8 pcnt_width = annotate_browser__pcnt_width(ab);

	/* PLT symbols contain external offsets */
	if (strstr(sym->name, "@plt"))
		return;

	if (!disasm_line__is_valid_jump(cursor, sym))
		return;

	target = ab->offsets[cursor->ops.target.offset];

	bcursor = browser_line(&cursor->al);
	btarget = browser_line(target);

	if (annotate_browser__opts.hide_src_code) {
		from = bcursor->idx_asm;
		to = btarget->idx_asm;
	} else {
		from = (u64)bcursor->idx;
		to = (u64)btarget->idx;
	}

	ui_browser__set_color(browser, HE_COLORSET_JUMP_ARROWS);
	__ui_browser__line_arrow(browser, pcnt_width + 2 + ab->addr_width,
				 from, to);

	if (is_fused(ab, cursor)) {
		ui_browser__mark_fused(browser,
				       pcnt_width + 3 + ab->addr_width,
				       from - 1,
				       to > from ? true : false);
	}
}

static unsigned int annotate_browser__refresh(struct ui_browser *browser)
{
	struct annotate_browser *ab = container_of(browser, struct annotate_browser, b);
	int ret = ui_browser__list_head_refresh(browser);
	int pcnt_width = annotate_browser__pcnt_width(ab);

	if (annotate_browser__opts.jump_arrows)
		annotate_browser__draw_current_jump(browser);

	ui_browser__set_color(browser, HE_COLORSET_NORMAL);
	__ui_browser__vline(browser, pcnt_width, 0, browser->height - 1);
	return ret;
}

static int disasm__cmp(struct annotation_line *a, struct annotation_line *b)
{
	int i;

	for (i = 0; i < a->samples_nr; i++) {
		if (a->samples[i].percent == b->samples[i].percent)
			continue;
		return a->samples[i].percent < b->samples[i].percent;
	}
	return 0;
}

static void disasm_rb_tree__insert(struct rb_root *root, struct annotation_line *al)
{
	struct rb_node **p = &root->rb_node;
	struct rb_node *parent = NULL;
	struct annotation_line *l;

	while (*p != NULL) {
		parent = *p;
		l = rb_entry(parent, struct annotation_line, rb_node);

		if (disasm__cmp(al, l))
			p = &(*p)->rb_left;
		else
			p = &(*p)->rb_right;
	}
	rb_link_node(&al->rb_node, parent, p);
	rb_insert_color(&al->rb_node, root);
}

static void annotate_browser__set_top(struct annotate_browser *browser,
				      struct annotation_line *pos, u32 idx)
{
	unsigned back;

	ui_browser__refresh_dimensions(&browser->b);
	back = browser->b.height / 2;
	browser->b.top_idx = browser->b.index = idx;

	while (browser->b.top_idx != 0 && back != 0) {
		pos = list_entry(pos->node.prev, struct annotation_line, node);

		if (disasm_line__filter(&browser->b, &pos->node))
			continue;

		--browser->b.top_idx;
		--back;
	}

	browser->b.top = pos;
	browser->b.navkeypressed = true;
}

static void annotate_browser__set_rb_top(struct annotate_browser *browser,
					 struct rb_node *nd)
{
	struct browser_line *bpos;
	struct annotation_line *pos;
	u32 idx;

	pos = rb_entry(nd, struct annotation_line, rb_node);
	bpos = browser_line(pos);

	idx = bpos->idx;
	if (annotate_browser__opts.hide_src_code)
		idx = bpos->idx_asm;
	annotate_browser__set_top(browser, pos, idx);
	browser->curr_hot = nd;
}

static void annotate_browser__calc_percent(struct annotate_browser *browser,
					   struct perf_evsel *evsel)
{
	struct map_symbol *ms = browser->b.priv;
	struct symbol *sym = ms->sym;
	struct annotation *notes = symbol__annotation(sym);
	struct disasm_line *pos;

	browser->entries = RB_ROOT;

	pthread_mutex_lock(&notes->lock);

	symbol__calc_percent(sym, evsel);

	list_for_each_entry(pos, &notes->src->source, al.node) {
		double max_percent = 0.0;
		int i;

		if (pos->al.offset == -1) {
			RB_CLEAR_NODE(&pos->al.rb_node);
			continue;
		}

		for (i = 0; i < pos->al.samples_nr; i++) {
			struct annotation_data *sample = &pos->al.samples[i];

			if (max_percent < sample->percent)
				max_percent = sample->percent;
		}

		if (max_percent < 0.01 && pos->al.ipc == 0) {
			RB_CLEAR_NODE(&pos->al.rb_node);
			continue;
		}
		disasm_rb_tree__insert(&browser->entries, &pos->al);
	}
	pthread_mutex_unlock(&notes->lock);

	browser->curr_hot = rb_last(&browser->entries);
}

static bool annotate_browser__toggle_source(struct annotate_browser *browser)
{
	struct annotation_line *al;
	struct browser_line *bl;
	off_t offset = browser->b.index - browser->b.top_idx;

	browser->b.seek(&browser->b, offset, SEEK_CUR);
	al = list_entry(browser->b.top, struct annotation_line, node);
	bl = browser_line(al);

	if (annotate_browser__opts.hide_src_code) {
		if (bl->idx_asm < offset)
			offset = bl->idx;

		browser->b.nr_entries = browser->nr_entries;
		annotate_browser__opts.hide_src_code = false;
		browser->b.seek(&browser->b, -offset, SEEK_CUR);
		browser->b.top_idx = bl->idx - offset;
		browser->b.index = bl->idx;
	} else {
		if (bl->idx_asm < 0) {
			ui_helpline__puts("Only available for assembly lines.");
			browser->b.seek(&browser->b, -offset, SEEK_CUR);
			return false;
		}

		if (bl->idx_asm < offset)
			offset = bl->idx_asm;

		browser->b.nr_entries = browser->nr_asm_entries;
		annotate_browser__opts.hide_src_code = true;
		browser->b.seek(&browser->b, -offset, SEEK_CUR);
		browser->b.top_idx = bl->idx_asm - offset;
		browser->b.index = bl->idx_asm;
	}

	return true;
}

static void annotate_browser__init_asm_mode(struct annotate_browser *browser)
{
	ui_browser__reset_index(&browser->b);
	browser->b.nr_entries = browser->nr_asm_entries;
}

#define SYM_TITLE_MAX_SIZE (PATH_MAX + 64)

static int sym_title(struct symbol *sym, struct map *map, char *title,
		     size_t sz)
{
	return snprintf(title, sz, "%s  %s", sym->name, map->dso->long_name);
}

static bool annotate_browser__callq(struct annotate_browser *browser,
				    struct perf_evsel *evsel,
				    struct hist_browser_timer *hbt)
{
	struct map_symbol *ms = browser->b.priv;
	struct disasm_line *dl = disasm_line(browser->selection);
	struct annotation *notes;
	struct addr_map_symbol target = {
		.map = ms->map,
		.addr = map__objdump_2mem(ms->map, dl->ops.target.addr),
	};
	char title[SYM_TITLE_MAX_SIZE];

	if (!ins__is_call(&dl->ins))
		return false;

	if (map_groups__find_ams(&target) ||
	    map__rip_2objdump(target.map, target.map->map_ip(target.map,
							     target.addr)) !=
	    dl->ops.target.addr) {
		ui_helpline__puts("The called function was not found.");
		return true;
	}

	notes = symbol__annotation(target.sym);
	pthread_mutex_lock(&notes->lock);

	if (notes->src == NULL && symbol__alloc_hist(target.sym) < 0) {
		pthread_mutex_unlock(&notes->lock);
		ui__warning("Not enough memory for annotating '%s' symbol!\n",
			    target.sym->name);
		return true;
	}

	pthread_mutex_unlock(&notes->lock);
	symbol__tui_annotate(target.sym, target.map, evsel, hbt);
	sym_title(ms->sym, ms->map, title, sizeof(title));
	ui_browser__show_title(&browser->b, title);
	return true;
}

static
struct disasm_line *annotate_browser__find_offset(struct annotate_browser *browser,
					  s64 offset, s64 *idx)
{
	struct map_symbol *ms = browser->b.priv;
	struct symbol *sym = ms->sym;
	struct annotation *notes = symbol__annotation(sym);
	struct disasm_line *pos;

	*idx = 0;
	list_for_each_entry(pos, &notes->src->source, al.node) {
		if (pos->al.offset == offset)
			return pos;
		if (!disasm_line__filter(&browser->b, &pos->al.node))
			++*idx;
	}

	return NULL;
}

static bool annotate_browser__jump(struct annotate_browser *browser)
{
	struct disasm_line *dl = disasm_line(browser->selection);
	u64 offset;
	s64 idx;

	if (!ins__is_jump(&dl->ins))
		return false;

	offset = dl->ops.target.offset;
	dl = annotate_browser__find_offset(browser, offset, &idx);
	if (dl == NULL) {
		ui_helpline__printf("Invalid jump offset: %" PRIx64, offset);
		return true;
	}

	annotate_browser__set_top(browser, &dl->al, idx);

	return true;
}

static
struct annotation_line *annotate_browser__find_string(struct annotate_browser *browser,
					  char *s, s64 *idx)
{
	struct map_symbol *ms = browser->b.priv;
	struct symbol *sym = ms->sym;
	struct annotation *notes = symbol__annotation(sym);
	struct annotation_line *al = browser->selection;

	*idx = browser->b.index;
	list_for_each_entry_continue(al, &notes->src->source, node) {
		if (disasm_line__filter(&browser->b, &al->node))
			continue;

		++*idx;

		if (al->line && strstr(al->line, s) != NULL)
			return al;
	}

	return NULL;
}

static bool __annotate_browser__search(struct annotate_browser *browser)
{
	struct annotation_line *al;
	s64 idx;

	al = annotate_browser__find_string(browser, browser->search_bf, &idx);
	if (al == NULL) {
		ui_helpline__puts("String not found!");
		return false;
	}

	annotate_browser__set_top(browser, al, idx);
	browser->searching_backwards = false;
	return true;
}

static
struct annotation_line *annotate_browser__find_string_reverse(struct annotate_browser *browser,
						  char *s, s64 *idx)
{
	struct map_symbol *ms = browser->b.priv;
	struct symbol *sym = ms->sym;
	struct annotation *notes = symbol__annotation(sym);
	struct annotation_line *al = browser->selection;

	*idx = browser->b.index;
	list_for_each_entry_continue_reverse(al, &notes->src->source, node) {
		if (disasm_line__filter(&browser->b, &al->node))
			continue;

		--*idx;

		if (al->line && strstr(al->line, s) != NULL)
			return al;
	}

	return NULL;
}

static bool __annotate_browser__search_reverse(struct annotate_browser *browser)
{
	struct annotation_line *al;
	s64 idx;

	al = annotate_browser__find_string_reverse(browser, browser->search_bf, &idx);
	if (al == NULL) {
		ui_helpline__puts("String not found!");
		return false;
	}

	annotate_browser__set_top(browser, al, idx);
	browser->searching_backwards = true;
	return true;
}

static bool annotate_browser__search_window(struct annotate_browser *browser,
					    int delay_secs)
{
	if (ui_browser__input_window("Search", "String: ", browser->search_bf,
				     "ENTER: OK, ESC: Cancel",
				     delay_secs * 2) != K_ENTER ||
	    !*browser->search_bf)
		return false;

	return true;
}

static bool annotate_browser__search(struct annotate_browser *browser, int delay_secs)
{
	if (annotate_browser__search_window(browser, delay_secs))
		return __annotate_browser__search(browser);

	return false;
}

static bool annotate_browser__continue_search(struct annotate_browser *browser,
					      int delay_secs)
{
	if (!*browser->search_bf)
		return annotate_browser__search(browser, delay_secs);

	return __annotate_browser__search(browser);
}

static bool annotate_browser__search_reverse(struct annotate_browser *browser,
					   int delay_secs)
{
	if (annotate_browser__search_window(browser, delay_secs))
		return __annotate_browser__search_reverse(browser);

	return false;
}

static
bool annotate_browser__continue_search_reverse(struct annotate_browser *browser,
					       int delay_secs)
{
	if (!*browser->search_bf)
		return annotate_browser__search_reverse(browser, delay_secs);

	return __annotate_browser__search_reverse(browser);
}

static void annotate_browser__update_addr_width(struct annotate_browser *browser)
{
	if (annotate_browser__opts.use_offset)
		browser->target_width = browser->min_addr_width;
	else
		browser->target_width = browser->max_addr_width;

	browser->addr_width = browser->target_width;

	if (annotate_browser__opts.show_nr_jumps)
		browser->addr_width += browser->jumps_width + 1;
}

static int annotate_browser__run(struct annotate_browser *browser,
				 struct perf_evsel *evsel,
				 struct hist_browser_timer *hbt)
{
	struct rb_node *nd = NULL;
	struct map_symbol *ms = browser->b.priv;
	struct symbol *sym = ms->sym;
	const char *help = "Press 'h' for help on key bindings";
	int delay_secs = hbt ? hbt->refresh : 0;
	int key;
	char title[SYM_TITLE_MAX_SIZE];

	sym_title(sym, ms->map, title, sizeof(title));
	if (ui_browser__show(&browser->b, title, help) < 0)
		return -1;

	annotate_browser__calc_percent(browser, evsel);

	if (browser->curr_hot) {
		annotate_browser__set_rb_top(browser, browser->curr_hot);
		browser->b.navkeypressed = false;
	}

	nd = browser->curr_hot;

	while (1) {
		key = ui_browser__run(&browser->b, delay_secs);

		if (delay_secs != 0) {
			annotate_browser__calc_percent(browser, evsel);
			/*
			 * Current line focus got out of the list of most active
			 * lines, NULL it so that if TAB|UNTAB is pressed, we
			 * move to curr_hot (current hottest line).
			 */
			if (nd != NULL && RB_EMPTY_NODE(nd))
				nd = NULL;
		}

		switch (key) {
		case K_TIMER:
			if (hbt)
				hbt->timer(hbt->arg);

			if (delay_secs != 0)
				symbol__annotate_decay_histogram(sym, evsel->idx);
			continue;
		case K_TAB:
			if (nd != NULL) {
				nd = rb_prev(nd);
				if (nd == NULL)
					nd = rb_last(&browser->entries);
			} else
				nd = browser->curr_hot;
			break;
		case K_UNTAB:
			if (nd != NULL) {
				nd = rb_next(nd);
				if (nd == NULL)
					nd = rb_first(&browser->entries);
			} else
				nd = browser->curr_hot;
			break;
		case K_F1:
		case 'h':
			ui_browser__help_window(&browser->b,
		"UP/DOWN/PGUP\n"
		"PGDN/SPACE    Navigate\n"
		"q/ESC/CTRL+C  Exit\n\n"
		"ENTER         Go to target\n"
		"ESC           Exit\n"
		"H             Go to hottest instruction\n"
		"TAB/shift+TAB Cycle thru hottest instructions\n"
		"j             Toggle showing jump to target arrows\n"
		"J             Toggle showing number of jump sources on targets\n"
		"n             Search next string\n"
		"o             Toggle disassembler output/simplified view\n"
		"s             Toggle source code view\n"
		"t             Circulate percent, total period, samples view\n"
		"/             Search string\n"
		"k             Toggle line numbers\n"
		"r             Run available scripts\n"
		"?             Search string backwards\n");
			continue;
		case 'r':
			{
				script_browse(NULL);
				continue;
			}
		case 'k':
			annotate_browser__opts.show_linenr =
				!annotate_browser__opts.show_linenr;
			break;
		case 'H':
			nd = browser->curr_hot;
			break;
		case 's':
			if (annotate_browser__toggle_source(browser))
				ui_helpline__puts(help);
			continue;
		case 'o':
			annotate_browser__opts.use_offset = !annotate_browser__opts.use_offset;
			annotate_browser__update_addr_width(browser);
			continue;
		case 'j':
			annotate_browser__opts.jump_arrows = !annotate_browser__opts.jump_arrows;
			continue;
		case 'J':
			annotate_browser__opts.show_nr_jumps = !annotate_browser__opts.show_nr_jumps;
			annotate_browser__update_addr_width(browser);
			continue;
		case '/':
			if (annotate_browser__search(browser, delay_secs)) {
show_help:
				ui_helpline__puts(help);
			}
			continue;
		case 'n':
			if (browser->searching_backwards ?
			    annotate_browser__continue_search_reverse(browser, delay_secs) :
			    annotate_browser__continue_search(browser, delay_secs))
				goto show_help;
			continue;
		case '?':
			if (annotate_browser__search_reverse(browser, delay_secs))
				goto show_help;
			continue;
		case 'D': {
			static int seq;
			ui_helpline__pop();
			ui_helpline__fpush("%d: nr_ent=%d, height=%d, idx=%d, top_idx=%d, nr_asm_entries=%d",
					   seq++, browser->b.nr_entries,
					   browser->b.height,
					   browser->b.index,
					   browser->b.top_idx,
					   browser->nr_asm_entries);
		}
			continue;
		case K_ENTER:
		case K_RIGHT:
		{
			struct disasm_line *dl = disasm_line(browser->selection);

			if (browser->selection == NULL)
				ui_helpline__puts("Huh? No selection. Report to linux-kernel@vger.kernel.org");
			else if (browser->selection->offset == -1)
				ui_helpline__puts("Actions are only available for assembly lines.");
			else if (!dl->ins.ops)
				goto show_sup_ins;
			else if (ins__is_ret(&dl->ins))
				goto out;
			else if (!(annotate_browser__jump(browser) ||
				     annotate_browser__callq(browser, evsel, hbt))) {
show_sup_ins:
				ui_helpline__puts("Actions are only available for function call/return & jump/branch instructions.");
			}
			continue;
		}
		case 't':
			if (annotate_browser__opts.show_total_period) {
				annotate_browser__opts.show_total_period = false;
				annotate_browser__opts.show_nr_samples = true;
			} else if (annotate_browser__opts.show_nr_samples)
				annotate_browser__opts.show_nr_samples = false;
			else
				annotate_browser__opts.show_total_period = true;
			annotate_browser__update_addr_width(browser);
			continue;
		case K_LEFT:
		case K_ESC:
		case 'q':
		case CTRL('c'):
			goto out;
		default:
			continue;
		}

		if (nd != NULL)
			annotate_browser__set_rb_top(browser, nd);
	}
out:
	ui_browser__hide(&browser->b);
	return key;
}

int map_symbol__tui_annotate(struct map_symbol *ms, struct perf_evsel *evsel,
			     struct hist_browser_timer *hbt)
{
	/* Set default value for show_total_period and show_nr_samples  */
	annotate_browser__opts.show_total_period =
		symbol_conf.show_total_period;
	annotate_browser__opts.show_nr_samples =
		symbol_conf.show_nr_samples;

	return symbol__tui_annotate(ms->sym, ms->map, evsel, hbt);
}

int hist_entry__tui_annotate(struct hist_entry *he, struct perf_evsel *evsel,
			     struct hist_browser_timer *hbt)
{
	/* reset abort key so that it can get Ctrl-C as a key */
	SLang_reset_tty();
	SLang_init_tty(0, 0, 0);

	return map_symbol__tui_annotate(&he->ms, evsel, hbt);
}


static unsigned count_insn(struct annotate_browser *browser, u64 start, u64 end)
{
	unsigned n_insn = 0;
	u64 offset;

	for (offset = start; offset <= end; offset++) {
		if (browser->offsets[offset])
			n_insn++;
	}
	return n_insn;
}

static void count_and_fill(struct annotate_browser *browser, u64 start, u64 end,
			   struct cyc_hist *ch)
{
	unsigned n_insn;
	u64 offset;

	n_insn = count_insn(browser, start, end);
	if (n_insn && ch->num && ch->cycles) {
		float ipc = n_insn / ((double)ch->cycles / (double)ch->num);

		/* Hide data when there are too many overlaps. */
		if (ch->reset >= 0x7fff || ch->reset >= ch->num / 2)
			return;

		for (offset = start; offset <= end; offset++) {
			struct annotation_line *al = browser->offsets[offset];

			if (al)
				al->ipc = ipc;
		}
	}
}

/*
 * This should probably be in util/annotate.c to share with the tty
 * annotate, but right now we need the per byte offsets arrays,
 * which are only here.
 */
static void annotate__compute_ipc(struct annotate_browser *browser, size_t size,
			   struct symbol *sym)
{
	u64 offset;
	struct annotation *notes = symbol__annotation(sym);

	if (!notes->src || !notes->src->cycles_hist)
		return;

	pthread_mutex_lock(&notes->lock);
	for (offset = 0; offset < size; ++offset) {
		struct cyc_hist *ch;

		ch = &notes->src->cycles_hist[offset];
		if (ch && ch->cycles) {
			struct annotation_line *al;

			if (ch->have_start)
				count_and_fill(browser, ch->start, offset, ch);
			al = browser->offsets[offset];
			if (al && ch->num_aggr)
				al->cycles = ch->cycles_aggr / ch->num_aggr;
			browser->have_cycles = true;
		}
	}
	pthread_mutex_unlock(&notes->lock);
}

static void annotate_browser__mark_jump_targets(struct annotate_browser *browser,
						size_t size)
{
	u64 offset;
	struct map_symbol *ms = browser->b.priv;
	struct symbol *sym = ms->sym;

	/* PLT symbols contain external offsets */
	if (strstr(sym->name, "@plt"))
		return;

	for (offset = 0; offset < size; ++offset) {
		struct annotation_line *al = browser->offsets[offset];
		struct disasm_line *dl;
		struct browser_line *blt;

		dl = disasm_line(al);

		if (!disasm_line__is_valid_jump(dl, sym))
			continue;

		al = browser->offsets[dl->ops.target.offset];

		/*
 		 * FIXME: Oops, no jump target? Buggy disassembler? Or do we
 		 * have to adjust to the previous offset?
 		 */
		if (al == NULL)
			continue;

		blt = browser_line(al);
		if (++blt->jump_sources > browser->max_jump_sources)
			browser->max_jump_sources = blt->jump_sources;

		++browser->nr_jumps;
	}
}

static inline int width_jumps(int n)
{
	if (n >= 100)
		return 5;
	if (n / 10)
		return 2;
	return 1;
}

int symbol__tui_annotate(struct symbol *sym, struct map *map,
			 struct perf_evsel *evsel,
			 struct hist_browser_timer *hbt)
{
	struct annotation_line *al;
	struct annotation *notes;
	size_t size;
	struct map_symbol ms = {
		.map = map,
		.sym = sym,
	};
	struct annotate_browser browser = {
		.b = {
			.refresh = annotate_browser__refresh,
			.seek	 = ui_browser__list_head_seek,
			.write	 = annotate_browser__write,
			.filter  = disasm_line__filter,
			.priv	 = &ms,
			.use_navkeypressed = true,
		},
	};
	int ret = -1, err;
	int nr_pcnt = 1;

	if (sym == NULL)
		return -1;

	size = symbol__size(sym);

	if (map->dso->annotate_warned)
		return -1;

	browser.offsets = zalloc(size * sizeof(struct annotation_line *));
	if (browser.offsets == NULL) {
		ui__error("Not enough memory!");
		return -1;
	}

	if (perf_evsel__is_group_event(evsel))
		nr_pcnt = evsel->nr_members;

	err = symbol__annotate(sym, map, evsel, sizeof(struct browser_line), &browser.arch);
	if (err) {
		char msg[BUFSIZ];
		symbol__strerror_disassemble(sym, map, err, msg, sizeof(msg));
		ui__error("Couldn't annotate %s:\n%s", sym->name, msg);
		goto out_free_offsets;
	}

	symbol__calc_percent(sym, evsel);

	ui_helpline__push("Press ESC to exit");

	notes = symbol__annotation(sym);
	browser.start = map__rip_2objdump(map, sym->start);

	list_for_each_entry(al, &notes->src->source, node) {
		struct browser_line *bpos;
		size_t line_len = strlen(al->line);

		if (browser.b.width < line_len)
			browser.b.width = line_len;
		bpos = browser_line(al);
		bpos->idx = browser.nr_entries++;
		if (al->offset != -1) {
			bpos->idx_asm = browser.nr_asm_entries++;
			/*
			 * FIXME: short term bandaid to cope with assembly
			 * routines that comes with labels in the same column
			 * as the address in objdump, sigh.
			 *
			 * E.g. copy_user_generic_unrolled
 			 */
			if (al->offset < (s64)size)
				browser.offsets[al->offset] = al;
		} else
			bpos->idx_asm = -1;
	}

	annotate_browser__mark_jump_targets(&browser, size);
	annotate__compute_ipc(&browser, size, sym);

	browser.addr_width = browser.target_width = browser.min_addr_width = hex_width(size);
	browser.max_addr_width = hex_width(sym->end);
	browser.jumps_width = width_jumps(browser.max_jump_sources);
	browser.nr_events = nr_pcnt;
	browser.b.nr_entries = browser.nr_entries;
	browser.b.entries = &notes->src->source,
	browser.b.width += 18; /* Percentage */

	if (annotate_browser__opts.hide_src_code)
		annotate_browser__init_asm_mode(&browser);

	annotate_browser__update_addr_width(&browser);

	ret = annotate_browser__run(&browser, evsel, hbt);

	annotated_source__purge(notes->src);

out_free_offsets:
	free(browser.offsets);
	return ret;
}

#define ANNOTATE_CFG(n) \
	{ .name = #n, .value = &annotate_browser__opts.n, }

/*
 * Keep the entries sorted, they are bsearch'ed
 */
static struct annotate_config {
	const char *name;
	bool *value;
} annotate__configs[] = {
	ANNOTATE_CFG(hide_src_code),
	ANNOTATE_CFG(jump_arrows),
	ANNOTATE_CFG(show_linenr),
	ANNOTATE_CFG(show_nr_jumps),
	ANNOTATE_CFG(show_nr_samples),
	ANNOTATE_CFG(show_total_period),
	ANNOTATE_CFG(use_offset),
};

#undef ANNOTATE_CFG

static int annotate_config__cmp(const void *name, const void *cfgp)
{
	const struct annotate_config *cfg = cfgp;

	return strcmp(name, cfg->name);
}

static int annotate__config(const char *var, const char *value,
			    void *data __maybe_unused)
{
	struct annotate_config *cfg;
	const char *name;

	if (!strstarts(var, "annotate."))
		return 0;

	name = var + 9;
	cfg = bsearch(name, annotate__configs, ARRAY_SIZE(annotate__configs),
		      sizeof(struct annotate_config), annotate_config__cmp);

	if (cfg == NULL)
		ui__warning("%s variable unknown, ignoring...", var);
	else
		*cfg->value = perf_config_bool(name, value);
	return 0;
}

void annotate_browser__init(void)
{
	perf_config(annotate__config, NULL);
}
