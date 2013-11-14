/*
 * Copyright © 2008-2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *    Chris Wilson <chris@chris-wilson.co.uk>
 *
 */

#include "drmP.h"
#include "drm.h"
#include "i915_drm.h"
#include "i915_drv.h"

/*
 * The BIOS typically reserves some of the system's memory for the exclusive
 * use of the integrated graphics. This memory is no longer available for
 * use by the OS and so the user finds that his system has less memory
 * available than he put in. We refer to this memory as stolen.
 *
 * The BIOS will allocate its framebuffer from the stolen memory. Our
 * goal is try to reuse that object for our own fbcon which must always
 * be available for panics. Anything else we can reuse the stolen memory
 * for is a boon.
 */

#define PTE_ADDRESS_MASK		0xfffff000
#define PTE_ADDRESS_MASK_HIGH		0x000000f0 /* i915+ */
#define PTE_MAPPING_TYPE_UNCACHED	(0 << 1)
#define PTE_MAPPING_TYPE_DCACHE		(1 << 1) /* i830 only */
#define PTE_MAPPING_TYPE_CACHED		(3 << 1)
#define PTE_MAPPING_TYPE_MASK		(3 << 1)
#define PTE_VALID			(1 << 0)

/**
 * i915_stolen_to_phys - take an offset into stolen memory and turn it into
 *                       a physical one
 * @dev: drm device
 * @offset: address to translate
 *
 * Some chip functions require allocations from stolen space and need the
 * physical address of the memory in question.
 */
static unsigned long i915_stolen_to_phys(struct drm_device *dev, u32 offset)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct pci_dev *pdev = dev->pdev;
	u32 base;

#if 0
	/* On the machines I have tested the Graphics Base of Stolen Memory
	 * is unreliable, so compute the base by subtracting the stolen memory
	 * from the Top of Low Usable DRAM which is where the BIOS places
	 * the graphics stolen memory.
	 */
	if (INTEL_INFO(dev)->gen > 3 || IS_G33(dev)) {
		/* top 32bits are reserved = 0 */
		pci_read_config_dword(pdev, 0xA4, &base);
	} else {
		/* XXX presume 8xx is the same as i915 */
		pci_bus_read_config_dword(pdev->bus, 2, 0x5C, &base);
	}
#else
	if (INTEL_INFO(dev)->gen >= 6) {
		/*SNB and above, Stolen memory base is available at
		offset 0x5c */
		u32 val = 0;

		pci_read_config_dword(pdev, 0x5C, &val);

		/*bits[31:20] has the value */
		base = val & 0xFFF00000;

	} else {
		if (INTEL_INFO(dev)->gen > 3 || IS_G33(dev)) {
			u16 val;
			pci_read_config_word(pdev, 0xb0, &val);
			base = val >> 4 << 20;
		} else {
			u8 val;
			pci_read_config_byte(pdev, 0x9c, &val);
			base = val >> 3 << 27;
		}
		base -= dev_priv->mm.gtt->stolen_size;
	}

#endif

	return base + offset;
}

static void i915_warn_stolen(struct drm_device *dev)
{
	DRM_INFO("not enough stolen space for compressed buffer, disabling\n");
	DRM_INFO("hint: you may be able to increase stolen memory size in the BIOS to avoid this\n");
}

static void i915_setup_compression(struct drm_device *dev, int size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mm_node *compressed_fb, *compressed_llb;
	unsigned long cfb_base;
	unsigned long ll_base = 0;

	/* initializing compressed_llb with NULL */
	compressed_llb = NULL;

	/* Just in case the BIOS is doing something questionable. */
	intel_disable_fbc(dev);

	compressed_fb = drm_mm_search_free(&dev_priv->mm.stolen, size, 4096, 0);
	if (compressed_fb)
		compressed_fb = drm_mm_get_block(compressed_fb, size, 4096);
	if (!compressed_fb)
		goto err;

	cfb_base = i915_stolen_to_phys(dev, compressed_fb->start);
	if (!cfb_base)
		goto err_fb;

	if (!(IS_GM45(dev) || HAS_PCH_SPLIT(dev))) {
		compressed_llb = drm_mm_search_free(&dev_priv->mm.stolen,
						    4096, 4096, 0);
		if (compressed_llb)
			compressed_llb = drm_mm_get_block(compressed_llb,
							  4096, 4096);
		if (!compressed_llb)
			goto err_fb;

		ll_base = i915_stolen_to_phys(dev, compressed_llb->start);
		if (!ll_base)
			goto err_llb;
	}

	dev_priv->cfb_size = size;

	dev_priv->compressed_fb = compressed_fb;
	if (HAS_PCH_SPLIT(dev))
		I915_WRITE(ILK_DPFC_CB_BASE, compressed_fb->start);
	else if (IS_GM45(dev)) {
		I915_WRITE(DPFC_CB_BASE, compressed_fb->start);
	} else {
		I915_WRITE(FBC_CFB_BASE, cfb_base);
		I915_WRITE(FBC_LL_BASE, ll_base);
		dev_priv->compressed_llb = compressed_llb;
	}

	DRM_DEBUG_KMS("FBC base 0x%08lx, ll base 0x%08lx, size %dM\n",
		      cfb_base, ll_base, size >> 20);
	return;

err_llb:
	drm_mm_put_block(compressed_llb);
err_fb:
	drm_mm_put_block(compressed_fb);
err:
	dev_priv->no_fbc_reason = FBC_STOLEN_TOO_SMALL;
	i915_warn_stolen(dev);
}

static void i915_cleanup_compression(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	drm_mm_put_block(dev_priv->compressed_fb);
	if (dev_priv->compressed_llb)
		drm_mm_put_block(dev_priv->compressed_llb);
}

static void i915_setup_pctx(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mm_node *pctx;
	unsigned int pctx_paddr;
	int pctx_size = 24*1024;

	if (!dev_priv->need_pcbr_setup) {
		/* BIOS set it up already, grab the pre-alloc'd space */
		int pcbr_offset;

		pctx_paddr = I915_READ(VLV_PCBR);
		DRM_DEBUG_DRIVER("PhysAddr of PwrCtx given by BIOS is 0x%x\n",
			pctx_paddr);

		/*BUG_ON((pctx_paddr >> 20) == 0);*/

		if ((pctx_paddr < dev_priv->mm.stolen_base) ||
		    (pctx_paddr > (dev_priv->mm.stolen_base +
				dev_priv->mm.gtt->stolen_size)) ||
		    ((pctx_paddr + pctx_size) > (dev_priv->mm.stolen_base +
					     dev_priv->mm.gtt->stolen_size))) {
			DRM_ERROR("PwrCtx not allocatd from stolen by BIOS\n");
			return;
		}

		pcbr_offset = (pctx_paddr & (~4095)) - dev_priv->mm.stolen_base;
		pctx = i915_reserve_stolen_for_preallocated(dev_priv->dev,
								pcbr_offset,
								pctx_size);
		if (!pctx)
			goto err;

		dev_priv->vlv_pctx = pctx;
		return;
	}

	pctx = drm_mm_search_free(&dev_priv->mm.stolen, pctx_size, 4096, 0);
	if (pctx)
		pctx = drm_mm_get_block(pctx, pctx_size, 4096);
	if (!pctx)
		goto err;

	pctx_paddr = i915_stolen_to_phys(dev, pctx->start);
	if (!pctx_paddr)
		goto err_free_pctx;

	dev_priv->vlv_pctx = pctx;
	I915_WRITE(VLV_PCBR, pctx_paddr);

	DRM_DEBUG_DRIVER("PhyAddr of PwrCtx allocated by Driver is 0x%x\n",
			pctx_paddr);
	return;

err_free_pctx:
	drm_mm_put_block(pctx);
err:
	DRM_DEBUG("not enough stolen space for PCTX, disabling\n");
}

static void i915_cleanup_pctx(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/*
	Clear out pctx only if it is setup by driver. Other case is
	that BIOS can do this setup
	*/
	if (dev_priv->vlv_pctx) {
		I915_WRITE(VLV_PCBR, 0);
		drm_mm_put_block(dev_priv->vlv_pctx);
	}
}

void i915_gem_cleanup_stolen(struct drm_device *dev)
{
	if (I915_HAS_FBC(dev) && i915_powersave)
		i915_cleanup_compression(dev);
	if (IS_VALLEYVIEW(dev) && i915_powersave)
		i915_cleanup_pctx(dev);
}

int i915_gem_init_stolen(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long prealloc_size = dev_priv->mm.gtt->stolen_size;
	int bios_reserved = 0;

	dev_priv->mm.stolen_base = i915_stolen_to_phys(dev, 0);
	if (dev_priv->mm.stolen_base == 0) {
		DRM_ERROR("Stolen Base detected as NULL\n");
		return 0;
	}

	DRM_DEBUG_DRIVER("The Base address of the Stolen area is 0x%lx\n",
			dev_priv->mm.stolen_base);

	if (IS_VALLEYVIEW(dev))
		bios_reserved = 1024*1024; /* top 1MB on VLV/BYT */
	BUG_ON(bios_reserved > dev_priv->mm.gtt->stolen_size);
	prealloc_size -= bios_reserved;

	/* Basic memrange allocator for stolen space */
	drm_mm_init(&dev_priv->mm.stolen, 0, prealloc_size);

	/* Try to set up FBC with a reasonable compressed buffer size */
	if (I915_HAS_FBC(dev) && i915_powersave) {
		int cfb_size;

		/* Leave 1M for line length buffer & misc. */

		/* Try to get a 32M buffer... */
		if (prealloc_size > (36*1024*1024))
			cfb_size = 32*1024*1024;
		else /* fall back to 7/8 of the stolen space */
			cfb_size = prealloc_size * 7 / 8;
		i915_setup_compression(dev, cfb_size);
	}

	/*Setup PCBR only if FW hasnt set it up already or
	  mark the stolen space, already used by BIOS for pctx,
	  as reserved */
	if (IS_VALLEYVIEW(dev) && i915_powersave)
		i915_setup_pctx(dev);

	return 0;
}

static struct sg_table *
i915_pages_create_for_stolen(struct drm_device *dev,
			     u32 offset, u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct sg_table *st;
	struct scatterlist *sg;

	DRM_DEBUG_DRIVER("offset=0x%x, size=0x%x\n", offset, size);
	BUG_ON(offset > dev_priv->mm.gtt->stolen_size - size);

	/* We hide that we have no struct page backing our stolen object
	 * by wrapping the contiguous physical allocation with a fake
	 * dma mapping in a single scatterlist.
	 */

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return NULL;

	if (sg_alloc_table(st, 1, GFP_KERNEL)) {
		kfree(st);
		return NULL;
	}

	sg = st->sgl;
	sg->offset = offset;
	sg->length = size;

	sg_dma_address(sg) = (dma_addr_t)dev_priv->mm.stolen_base + offset;
	sg_dma_len(sg) = size;

	return st;
}

static int
i915_gem_object_get_pages_stolen(struct drm_i915_gem_object *obj,
				  struct page **pages,
				  gfp_t gfpmask,
				  u32 *offset)
{
	BUG();
	return -EINVAL;
}

static int i915_gem_object_put_pages_stolen(struct drm_i915_gem_object *obj)
{
	/* Should only be called during free */
	sg_free_table(obj->sg_table);
	kfree(obj->sg_table);
	return 0;
}

static const struct drm_i915_gem_object_ops i915_gem_object_stolen_ops = {
	.get_pages = i915_gem_object_get_pages_stolen,
	.put_pages = i915_gem_object_put_pages_stolen,
	.release   = i915_gem_object_release_stolen,
};

static struct drm_i915_gem_object *
_i915_gem_object_create_stolen(struct drm_device *dev,
			       struct drm_mm_node *stolen)
{
	struct drm_i915_gem_object *obj;

	/* Allocate the new object */
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (obj == NULL)
		return NULL;

	if (drm_gem_private_object_init(dev, &(obj->base), stolen->size))
		goto cleanup;

	i915_gem_object_init(dev, obj, &i915_gem_object_stolen_ops);

	obj->sg_table = i915_pages_create_for_stolen(dev,
						stolen->start, stolen->size);
	if (obj->sg_table == NULL)
		goto cleanup;

	obj->stolen = stolen;

	obj->base.write_domain = I915_GEM_DOMAIN_GTT;
	obj->base.read_domains = I915_GEM_DOMAIN_GTT;
	obj->cache_level = I915_CACHE_NONE;

	obj->pages = NULL;

	DRM_DEBUG_DRIVER("Created stolen obj: with offset=0x%lx, size=0x%lx\n",
			stolen->start, stolen->size);
	return obj;

cleanup:
	kfree(obj);
	return NULL;
}

struct drm_i915_gem_object *
i915_gem_object_create_stolen(struct drm_device *dev, u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj;
	struct drm_mm_node *stolen;

	if (dev_priv->mm.stolen_base == 0)
		return NULL;

	if (size == 0)
		return NULL;

	stolen = drm_mm_search_free(&dev_priv->mm.stolen, size, 4096, 0);
	if (stolen)
		stolen = drm_mm_get_block(stolen, size, 4096);
	if (stolen == NULL)
		return NULL;

	obj = _i915_gem_object_create_stolen(dev, stolen);
	if (obj)
		return obj;

	drm_mm_put_block(stolen);
	return NULL;
}

static void i915_memset_stolen_obj(struct drm_device *dev,
					struct drm_i915_gem_object *obj)
{
	if (obj->gtt_space == NULL) {
		int ret;
		char __iomem *base;
		int size = obj->base.size;
		struct drm_i915_private *dev_priv = obj->base.dev->dev_private;
		unsigned alignment = 0;
		bool map_and_fenceable =  true;

		ret = i915_gem_object_pin(obj, alignment, map_and_fenceable);
		if (ret) {
			DRM_ERROR("Mapping of User FB to GTT failed\n");
			return;
		}

		/* Get the CPU virtual address of the frame buffer */
		base =
		   ioremap_wc(dev_priv->mm.gtt_base_addr + obj->gtt_offset,
			size);
		if (base == NULL) {
			DRM_ERROR("Mapping of User FB to CPU failed\n");
			i915_gem_object_unpin(obj);
			return;
		}

		memset_io(base, 0, size);

		iounmap(base);
		i915_gem_object_unpin(obj);

		DRM_DEBUG_DRIVER(
		   "UserFB obj ptr=0x%x cleared using CPU address 0x%x\n",
			(u32)obj, (u32)base);
	} else
		BUG_ON(1);
}

void
i915_gem_object_move_to_stolen(struct drm_i915_gem_object *obj)
{
	struct drm_device *dev = obj->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mm_node *stolen;
	u32 size = obj->base.size;

	if (obj->stolen) {
		BUG_ON(obj->sg_table == NULL);
		return;
	}

	if (dev_priv->mm.stolen_base == 0)
		return;

	if (size == 0)
		return;

	stolen = drm_mm_search_free(&dev_priv->mm.stolen, size, 4096, 0);
	if (stolen)
		stolen = drm_mm_get_block(stolen, size, 4096);
	if (stolen == NULL) {
		DRM_DEBUG_DRIVER("ran out of stolen space\n");
		return;
	}
	/* Set up the object to use the stolen memory,
	 * backing store no longer managed by shmem layer */
	drm_gem_object_release(&(obj->base));
	obj->base.filp = NULL;
	obj->base.driver_private = (void *)&i915_gem_object_stolen_ops;

	obj->sg_table = i915_pages_create_for_stolen(dev,
						stolen->start, stolen->size);
	if (obj->sg_table == NULL)
		goto cleanup;

	obj->stolen = stolen;

	obj->base.write_domain = I915_GEM_DOMAIN_GTT;
	obj->base.read_domains = I915_GEM_DOMAIN_GTT;
	obj->cache_level = I915_CACHE_NONE;

	obj->pages = NULL;

	DRM_DEBUG_DRIVER("Obj moved to stolen with ptr=0x%x, size=%x\n",
					(unsigned int)obj, size);

	/* Any allocation from shmem area is also zeroed out.
	   Need to do the same for allocations from stolen area */
	i915_memset_stolen_obj(dev, obj);
	return;

cleanup:
	drm_mm_put_block(stolen);
	return;
}

struct drm_mm_node *
i915_reserve_stolen_for_preallocated(struct drm_device *dev,
					       u32 stolen_offset,
					       u32 size)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mm_node *stolen;

	if (dev_priv->mm.stolen_base == 0)
		return NULL;

	DRM_DEBUG_DRIVER("Creating preallocated stolen object:"\
					 "stolen_offset=%x, size=%x\n",
					 stolen_offset, size);

	/* KISS and expect everything to be page-aligned */
	BUG_ON(stolen_offset & 4095);
	BUG_ON(size & 4095);

	if (WARN_ON(size == 0))
		return NULL;

	stolen = drm_mm_create_block(&dev_priv->mm.stolen,
				     stolen_offset, size,
				     false);
	if (stolen == NULL) {
		DRM_DEBUG_DRIVER("failed to allocate stolen space\n");
		return NULL;
	}

	return stolen;
}

void
i915_gem_object_release_stolen(struct drm_i915_gem_object *obj)
{
	if (obj->stolen) {
		drm_mm_put_block(obj->stolen);
		obj->stolen = NULL;
	}
}
