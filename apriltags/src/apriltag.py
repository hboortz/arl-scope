# 4/1/2015
# Charles O. Goddard

import numpy
import cffi
import os.path


ffi = cffi.FFI()


ffi.cdef("""
	typedef struct zarray zarray_t;
	struct zarray
	{
	    size_t el_sz;
	    int size; 
	    int alloc;
	    char *data;
	};
	void zarray_get(const zarray_t *za, int idx, void *p);
	void zarray_get_volatile(const zarray_t *za, int idx, void **p);

	typedef struct apriltag_family apriltag_family_t;
	struct apriltag_family {
		uint32_t ncodes;
		char* name;
		...;
	};

	apriltag_family_t *tag36h11_create();
	void tag36h11_destroy(apriltag_family_t *tf);
	apriltag_family_t *tag36h10_create();
	void tag36h10_destroy(apriltag_family_t *tf);
	apriltag_family_t *tag25h9_create();
	void tag25h9_destroy(apriltag_family_t *tf);
	apriltag_family_t *tag25h7_create();
	void tag25h7_destroy(apriltag_family_t *tf);
	apriltag_family_t *tag16h5_create();
	void tag16h5_destroy(apriltag_family_t *tf);

	struct apriltag_quad_thresh_params
	{
	    int min_cluster_pixels;
	    int max_nmaxima;
	    float critical_rad;
	    float max_line_fit_mse;
	    int min_white_black_diff;
	    int deglitch;
	};

	struct apriltag_detector
	{
	    int nthreads;
	    float quad_decimate;
	    float quad_sigma;
	    int refine_edges;
	    int refine_decode;
	    int refine_pose;
	    int debug;
    	struct apriltag_quad_thresh_params qtp;
    	...;
	};
	typedef struct apriltag_detector apriltag_detector_t;

	apriltag_detector_t *apriltag_detector_create();
	void apriltag_detector_add_family(apriltag_detector_t *td,
									  apriltag_family_t *fam);
	void apriltag_detector_destroy(apriltag_detector_t *td);

	typedef struct
	{
	    int nrows, ncols;
	    double data[];
	} matd_t;
	typedef struct apriltag_detection apriltag_detection_t;
	struct apriltag_detection
	{
	    apriltag_family_t *family;
	    int id;
	    int hamming;
	    float goodness;
	    float decision_margin;
	    matd_t *H;
	    double c[2];
	    double p[4][2];
	};

	typedef struct image_u8 image_u8_t;
	struct image_u8
	{
	    const int width, height;
	    const int stride;

	    uint8_t *const buf; // const pointer, not buf
	};
	image_u8_t *image_u8_create(unsigned int width, unsigned int height);
	image_u8_t *image_u8_create_from_buf(int width, int height, uint8_t *rgb, int stride);
	void image_u8_destroy(image_u8_t *im);
	
	zarray_t *apriltag_detector_detect(apriltag_detector_t *td, image_u8_t *im_orig);
	void apriltag_detections_destroy(zarray_t *detections);
""")


source_dir = os.path.dirname(os.path.realpath(__file__))


sources = [os.path.join(source_dir, 'apriltag', x) for x in 
	('apriltag.c', 'apriltag_quad_thresh.c', 'tag16h5.c', 'tag25h7.c',
	'tag25h9.c', 'tag36h10.c', 'tag36h11.c', 'tag36artoolkit.c',
	'g2d.c', 'common/zarray.c', 'common/zhash.c', 'common/zmaxheap.c',
	'common/unionfind.c', 'common/matd.c', 'common/image_u8.c',
	'common/pnm.c', 'common/image_f32.c', 'common/image_u32.c',
	'common/workerpool.c', 'common/time_util.c', 'common/svd22.c',
	'common/homography.c', 'common/string_util.c', 'common/getopt.c')]


C = ffi.verify(
	"""
	#include <apriltag.h>
	#include <tag36h11.h>

	image_u8_t *image_u8_create_from_buf(int width, int height, uint8_t *buf, int stride)
	{
	    // const initializer
	    image_u8_t tmp = { .width = width, .height = height, .stride = stride, .buf = buf };

	    image_u8_t *im = calloc(1, sizeof(image_u8_t));
	    memcpy(im, &tmp, sizeof(image_u8_t));
	    return im;
	}
	""",
	sources=sources,
	include_dirs=[os.path.join(source_dir, 'apriltag'), os.path.join(source_dir, 'apriltag', 'common')],
	extra_compile_args=['-std=c99'])


class TagFamily(object):
	def __init__(self, cdata, free):
		self.cdata = cdata
		self.free = free
		self.name = ffi.string(cdata.name)

	def __del__(self):
		print 'Cleaning up %s family' % (self.name,)
		self.free(self.cdata)

	def __str__(self):
		return self.name
	def __repr__(self):
		return 'TagFamily(C.%s_create(), C.%s_destroy)' % (self.name, self.name)

	@property
	def ncodes(self):
	    return self.cdata.ncodes


tag36h11 = TagFamily(C.tag36h11_create(), C.tag36h11_destroy)
tag36h10 = TagFamily(C.tag36h10_create(), C.tag36h10_destroy)
tag25h9 = TagFamily(C.tag25h9_create(), C.tag25h9_destroy)
tag25h7 = TagFamily(C.tag25h7_create(), C.tag25h7_destroy)
tag16h5 = TagFamily(C.tag16h5_create(), C.tag16h5_destroy)


class Detector(object):
	def __init__(self, family):
		assert(isinstance(family, TagFamily))
		self.family = family
		self.cdata = C.apriltag_detector_create()
		C.apriltag_detector_add_family(self.cdata, family.cdata)

	def __del__(self):
		print 'Cleaning up %s detector' % (self.family.name,)
		C.apriltag_detector_destroy(self.cdata)

	def detect(self, image):
		img_c = C.image_u8_create_from_buf(image)
		res = DetectionArray(C.apriltag_detector_detect(self.cdata, img_c))
		C.image_u8_destroy(img_c)
		return res


class DetectionArray(object):
	def __init__(self, cdata):
		self.cdata = cdata

	def __len__(self):
		return self.cdata.size

	def __del__(self):
		C.apriltag_detections_destroy(self.cdata)

	def __getitem__(self, idx):
		assert(idx <= len(self))
		ptr = ffi.new("apriltag_detection_t*")
		C.zarray_get_volatile(self.cdata, idx, ffi.addressof(ptr))
		res = ptr[0]
		del ptr
		return Detection(res)


class Detection(object):
	def __init__(self, cdata):
		self.cdata = cdata

	@property
	def id(self):
		return self.cdata.id

	@property
	def hamming(self):
		return self.cdata.hamming

	@property
	def goodness(self):
		return self.cdata.goodness

	@property
	def decision_margin(self):
		return self.cdata.decision_margin

	@property
	def homography(self):
		mat = self.cdata.H[0]
		res = numpy.matrix(numpy.zeros(mat.nrows, mat.ncols, dtype=float))
		for row in range(mat.nrows):
			for col in range(mat.ncols):
				res[row, col] = mat.data[row*mat.ncols + col]
		return res

