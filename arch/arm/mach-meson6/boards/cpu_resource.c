#include <linux/kernel.h>
#include <linux/init.h>
#include "common-data.h"
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <mach/irqs.h>

static LIST_HEAD(cpu_module_class);
static struct cpu_module_class * mesonplat_cpu_resource_get_class(const char * name)
{
	struct cpu_module_class * cls;
	list_for_each_entry(cls,&cpu_module_class,list)
	{
		if(strcmp(name,cls->name))
			continue;
		return cls;
	}
	return NULL;
}
static bool compare_and_copy_class(struct cpu_module_class * a,struct cpu_module_class * b)
{
	if(a==b)
		return true;
	if(a->driver!=NULL&&b->driver!=NULL)
	{
		if(strcmp(a->driver,b->driver))
			return false;
		
	}
	if(a->driver==NULL)
		a->driver=b->driver;
	return true;
}
int32_t __init mesonplat_cpu_resource_add(struct cpu_module_class * cls,struct cpu_module_object* obj)
{
	struct cpu_module_class * p;
	struct cpu_module_object* first;
	int id;
	if(cls==NULL || obj==NULL || cls->name==NULL)
		return -ENXIO;
	p=mesonplat_cpu_resource_get_class(cls->name);
	if(p==NULL)
	{
		INIT_LIST_HEAD(&cls->child);
		list_add(&cls->list,&cpu_module_class);
		p=cls;
		id=0;
	}else{
		first=list_first_entry(&p->child,typeof(*first),list);
		id=first->device.id+1;
		if(compare_and_copy_class(p,cls)==false)
			return -ENXIO;
	}
	
	obj->device.name=cls->driver?cls->driver:cls->name;
	obj->device.id=id;
	list_add(&obj->list,&p->child);
	return 0;
}
int32_t mesonplat_cpu_resource_get_num(const char * name)
{
	struct cpu_module_class * cls;
	struct cpu_module_object *obj;
	int32_t ret=0;
	cls=mesonplat_cpu_resource_get_class(name);
	if(cls==NULL)
		return 0;
	list_for_each_entry(obj,&cls->child,list)
	{
		ret++;
	}
	return ret;
}
struct cpu_module_object * mesonplat_cpu_resource_get(const char * cls_name,const char * obj_name)
{
	struct cpu_module_class * cls;
	struct cpu_module_object *obj;
	if(cls_name==NULL || obj_name==NULL)
		return NULL;
	cls=mesonplat_cpu_resource_get_class(cls_name);
	if(cls==NULL)
	{
		return NULL;
	}
	list_for_each_entry(obj,&cls->child,list)
	{
		if(strcmp(obj_name,obj->name))
			continue;
		return obj;
	}
	return NULL;

}
int32_t mesonplat_resource_modify(const char * cls_name,const char * obj_name,	int id,
	struct resource	* resource,
	u32		num_resources,
	void * platform_data)
{
	
	struct cpu_module_object *obj;
	if(cls_name==NULL || obj_name==NULL)
		return -ENXIO;
	obj=mesonplat_cpu_resource_get(cls_name,obj_name);
	if(obj==NULL)
	{
		return -ENXIO;
	}
	obj->device.dev.platform_data=platform_data?platform_data:obj->device.dev.platform_data;
	if(resource)
	{
		obj->device.num_resources=num_resources;
		obj->device.resource=resource;
	}
	if(id>-2)
	{
		obj->device.id=id;
	}
	return 0;
}

static int32_t do_mesonplat_platform_device(const char * cls_name,const char * obj_name,	int id,
	struct resource	* resource,
	u32		num_resources,
	void * platform_data,bool early)
{
	int32_t ret;
	struct platform_device *devs[1];
	struct cpu_module_object *obj;
	if(cls_name==NULL || obj_name==NULL)
		return -ENXIO;
	obj=mesonplat_cpu_resource_get(cls_name,obj_name);
	if(obj==NULL)
	{
		return -ENXIO;
	}
	obj->device.dev.platform_data=platform_data?platform_data:obj->device.dev.platform_data;
	if(resource)
	{
		obj->device.num_resources=num_resources;
		obj->device.resource=resource;
	}
	if(id>-2)
	{
		obj->device.id=id;
	}
	devs[0]=&obj->device;
	if(early==true)
	{
		early_platform_add_devices(devs,1);
		return 0;
	}
	return platform_add_devices(devs,1);
}

int32_t mesonplat_register_device(const char * cls_name,const char * obj_name,void * platform_data)
{
	return do_mesonplat_platform_device(cls_name,obj_name,-2,NULL,0,platform_data,false);
}

int32_t mesonplat_register_device_early(const char * cls_name,const char * obj_name,void * platform_data)
{
	return do_mesonplat_platform_device(cls_name,obj_name,-2,NULL,0,platform_data,true);
}


#if 0
/**
 * End of Meson 3 uart resource description
 */
static struct cpu_resource res_table[]={
		AVAILABLE_UARTS(),
};
#define dbg_print(a...)  printk(a)

bool meson_platform_data_set_default(struct platform_device * pdev,const char * id);
int  meson_platform_data_set(struct platform_device * pdev,const char * id ,void * platform_data)
{
	int i;
	dbg_print("%s request name=%s id=%s\n",__func__,pdev->name,id);
	for(i=0;i<ARRAY_SIZE(res_table);i++)
	{
		
		if(strcmp(pdev->name,res_table[i].name)!=0 || (strcmp(id,res_table[i].id)!=0))
			continue;
		if(platform_data)
			pdev->dev.platform_data=platform_data;
		else if(meson_platform_data_set_default(pdev,id))
			return false;
		pdev->num_resources=res_table[i].num_resources;
		pdev->resource=res_table[i].resource;
		return	platform_device_register(pdev);
	}
	return false;
}
#endif
