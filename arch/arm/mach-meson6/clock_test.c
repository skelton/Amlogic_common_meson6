void dump_clock_tree(struct clk* clk);
void print_clk_name(struct clk* clk);

static int clk_disable_before(void* privdata)
{
	printk("[%s] :%s\n",__func__,privdata);
	return 0;
}
static int clk_disable_after(void* privdata,int failed)
{
	printk("[%s] :%s   failed:%d\n",__func__,privdata,failed);
	return 0;
}

static int clk_enable_before(void* privdata)
{
	printk("%s] :%s\n",__func__,privdata);
	return 0;
}
static int clk_enable_after(void* privdata,int failed)
{
	printk("[%s] :%s   failed:%d\n",__func__,privdata,failed);
	return 0;
}
static int clk_ratechange_before(unsigned long newrate,void* privdata)
{
	printk("[%s] :%s\n",__func__,privdata);
	return 0;
}
static int clk_ratechange_after(unsigned long newrate,void* privdata,int failed)
{
	printk("[%s] :%s   failed:%d\n",__func__,privdata,failed);
	return 0;
}

static struct clk_ops uart_clk_ops={
	.clk_disable_before = clk_disable_before,
	.clk_disable_after = clk_disable_after,
	.clk_enable_before = clk_enable_before,
	.clk_enable_after = clk_enable_after,
	.clk_ratechange_before = clk_ratechange_before,
	.clk_ratechange_after = clk_ratechange_after,
	.privdata = "clk81 -> uart",
};

static struct clk_ops usb_clk_ops={
	.clk_disable_before = clk_disable_before,
	.clk_disable_after = clk_disable_after,
	.clk_enable_before = clk_enable_before,
	.clk_enable_after = clk_enable_after,
	.clk_ratechange_before = clk_ratechange_before,
	.clk_ratechange_after = clk_ratechange_after,
	.privdata = "clk81 -> usb",
};

static struct clk_ops xtaldev_clk_ops={
	.clk_disable_before = clk_disable_before,
	.clk_disable_after = clk_disable_after,
	.clk_enable_before = clk_enable_before,
	.clk_enable_after = clk_enable_after,
	.clk_ratechange_before = clk_ratechange_before,
	.clk_ratechange_after = clk_ratechange_after,
	.privdata = "xtal device",
};

static struct clk_ops audio_clk_ops={
	.clk_disable_before = clk_disable_before,
	.clk_disable_after = clk_disable_after,
	.clk_enable_before = clk_enable_before,
	.clk_enable_after = clk_enable_after,
	.clk_ratechange_before = clk_ratechange_before,
	.clk_ratechange_after = clk_ratechange_after,
	.privdata = "fixed pll -> audio",
};
static struct clk_ops clk81_clk_ops={
	.clk_disable_before = clk_disable_before,
	.clk_disable_after = clk_disable_after,
	.clk_enable_before = clk_enable_before,
	.clk_enable_after = clk_enable_after,
	.clk_ratechange_before = clk_ratechange_before,
	.clk_ratechange_after = clk_ratechange_after,
	.privdata = "fixed pll -> clk81",
};

static struct clk_ops audiodev_clk_ops={
	.clk_disable_before = clk_disable_before,
	.clk_disable_after = clk_disable_after,
	.clk_enable_before = clk_enable_before,
	.clk_enable_after = clk_enable_after,
	.clk_ratechange_before = clk_ratechange_before,
	.clk_ratechange_after = clk_ratechange_after,
	.privdata = "audio device",
};

static int set_rate(struct clk *clk, unsigned long rate)
{
	printk("[ww: %s] rate:%d\n",__func__,rate);
	print_clk_name(clk);
	return 0;
}

void test_clock_tree()
{
	  clkdev_add(&clk_lookup_xtal);
    CLK_DEFINE(pll_ddr,xtal,3,NULL,clk_msr_get,NULL,NULL,NULL);
    PLL_CLK_DEFINE(sys,23);
    PLL_CLK_DEFINE(vid2,12);
    PLL_CLK_DEFINE(fixed,-1);
    PLL_CLK_DEFINE(hpll,-1);///@todo unknown now 
  
    // Add clk81
    CLK_DEFINE(clk81,pll_fixed,7,NULL,clk_msr_get,NULL,NULL,NULL);
    CLK_DEFINE(clkA9,pll_sys,-1,NULL,NULL,NULL,NULL,NULL);
    CLK_DEFINE(audio,pll_fixed,-1,NULL,NULL,NULL,NULL,NULL);
   
    PLL_RELATION_DEF(sys,xtal);
    PLL_RELATION_DEF(ddr,xtal);
    PLL_RELATION_DEF(vid2,xtal);
    PLL_RELATION_DEF(fixed,xtal);
    PLL_RELATION_DEF(hpll,xtal);
    
    CLK_PLL_CHILD_DEF(clkA9,sys);
    
    CLK_PLL_CHILD_DEF(clk81,fixed);
    CLK_PLL_CHILD_DEF(audio,fixed);   
 		
 		dump_clock_tree(&clk_xtal);
 		
 		printk("unregister clk clkA9 ...\n");
		clk_unregister(&clk_clkA9);
		dump_clock_tree(&clk_xtal);
		
 		printk("register clk clkA9 to pll_sys...\n");
		clk_register(&clk_clkA9,"pll_sys");
		dump_clock_tree(&clk_xtal);
		
		printk("unregister clk_audio ...\n");
		clk_unregister(&clk_audio);
		dump_clock_tree(&clk_xtal);
		
		printk("register clk_audio to pll_fixed ...\n");
		clk_register(&clk_audio,"pll_fixed");
		dump_clock_tree(&clk_xtal);
		
		printk("unregister clk_clk81 ...\n");
		clk_unregister(&clk_clk81);
		dump_clock_tree(&clk_xtal);
		
		printk("register clkA9, clk81, clk_audio ...\n");
		clk_register(&clk_clkA9,"pll_sys");
		clk_register(&clk_audio,"pll_fixed");
		clk_register(&clk_clk81,"pll_fixed");				
		dump_clock_tree(&clk_xtal);
		
		printk("register clk operations ...\n");
		clk_ops_register(&clk_pll_fixed,&audio_clk_ops);
		clk_ops_register(&clk_pll_fixed,&clk81_clk_ops);
		clk_ops_register(&clk_clk81,&uart_clk_ops);
		clk_ops_register(&clk_clk81,&usb_clk_ops);
		clk_ops_register(&clk_xtal,&xtaldev_clk_ops);
		clk_ops_register(&clk_audio,&audiodev_clk_ops);
		
		printk("disable xtal ...\n");
		clk_disable(&clk_xtal);
		
		printk("enable xtal ...\n");
		clk_enable(&clk_xtal);
		
		printk("disable pll_fixed ...\n");
		clk_disable(&clk_pll_fixed);
		
		printk("enable pll_fixed ...\n");
		clk_enable(&clk_pll_fixed);

	
		printk("disable clk_clk81 ...\n");
		clk_disable(&clk_clk81);
		
		printk("enable clk_clk81 ...\n");
		clk_enable(&clk_clk81);

		printk("disable clk_audio ...\n");
		clk_disable(&clk_audio);
		
		printk("enable clk_audio ...\n");
		clk_enable(&clk_audio);
		
		clk_xtal.set_rate = set_rate;
		clk_clk81.set_rate = set_rate;
		clk_audio.set_rate = set_rate;
		
		printk("change rate xtal ...\n");
		clk_set_rate(&clk_xtal,250000000);
		
		printk("change rate pll_fixed ...\n");
		clk_set_rate(&clk_pll_fixed,2000000000);
		
		printk("change rate clk81 ...\n");
		clk_set_rate(&clk_clk81,2000000000);

		printk("change rate clk audio ...\n");
		clk_set_rate(&clk_audio,2000000000);

}

extern struct clk_lookup * lookup_clk(struct clk* clk);
void print_clk_name(struct clk* clk)
{
		struct clk_lookup * p = lookup_clk(clk);
		if(p)
			printk("  %s  \n",p->dev_id);
		else
			printk(" unknown \n");
}

void dump_child(int nlevel, struct clk* clk)
{
		if(!IS_CLK_ERR(clk)){
			int i;
			for(i = 0; i < nlevel; i++)
				printk("  ");
			print_clk_name(clk);
			dump_child(nlevel+6,(struct clk*)(clk->child.next));
			{
				struct clk * p = (struct clk*)(clk->sibling.prev);
				while(p){
					for(i = 0; i < nlevel; i++)
						printk("  ");
					print_clk_name(p);
					dump_child(nlevel+6,(struct clk*)(p->child.next));
					p = (struct clk*)(p->sibling.prev);
				}
				
				p = (struct clk*)(clk->sibling.next);
				while(p){
					for(i = 0; i < nlevel; i++)
						printk("  ");
					print_clk_name(p);
					dump_child(nlevel+6,(struct clk*)(p->child.next));
					p = (struct clk*)(p->sibling.next);
				}
			}
		}
}

void dump_clock_tree(struct clk* clk)
{
	printk("========= dump clock tree==============\n");
	mutex_lock(&clock_ops_lock);

	int nlevel = 0;
	if(!IS_CLK_ERR(clk)){
		print_clk_name(clk);
		dump_child(nlevel + 6,(struct clk*)(clk->child.next));
			{	int i;
				struct clk * p = (struct clk*)clk->sibling.prev;
				while(p){
					for(i = 0; i < nlevel; i++)
						printk("  ");
					print_clk_name(p);
					dump_child(nlevel+6,(struct clk*)(p->child.next));
					p = (struct clk*)clk->sibling.prev;
				}
				
				p = (struct clk*)clk->sibling.next;
				while(p){
					for(i = 0; i < nlevel; i++)
						printk("  ");
					print_clk_name(p);
					dump_child(nlevel+6,(struct clk*)(p->child.next));
					p = (struct clk*)clk->sibling.next;
				}
			}
	}
	mutex_unlock(&clock_ops_lock);
	printk("========= dump clock tree end ==============\n");
}
