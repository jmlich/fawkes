
/***************************************************************************
 *  net.cpp - Camera to access images over the network
 *
 *  Created: Wed Feb 01 12:24:04 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <cams/net.h>
#include <cams/cam_exceptions.h>

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_image_content.h>
#include <fvutils/net/fuse_imagelist_content.h>
#include <fvutils/system/camargp.h>
#include <fvutils/compression/jpeg_decompressor.h>

#include <netinet/in.h>

#include <cstring>

/** @class NetworkCamera <cams/net.h>
 * Network camera.
 * Retrieve images via network (FUSE).
 * @see FuseClient
 * @author Tim Niemueller
 */

/** Constructor.
 * Allows to initiate a NetworkCamera w/o specifying an image id. This can be
 * done later with the set_image_id() method.
 * @param host host to connect to
 * @param port port to connect to
 * @param jpeg if true jpeg images will be transferred and automatically be
 * decompressed, otherwise raw images are transferred
 */
NetworkCamera::NetworkCamera(const char *host, unsigned short port, bool jpeg)
{
  if ( host == NULL ) {
    throw NullPointerException("NetworkCamera: host must not be NULL");
  }
  __image_id = 0;
  __host = strdup(host);
  __port = port;
  __get_jpeg = jpeg;

  __connected       = false;
  __local_version   = 0;
  __remote_version  = 0;
  __decompressor    = NULL;
  __decompressed_buffer = NULL;
  __last_width = 0;
  __last_height = 0;
  __fuse_image = NULL;
  __fuse_message = NULL;

  __fusec = new FuseClient(__host, __port, this);
  if ( __get_jpeg ) {
    __decompressor = new JpegImageDecompressor();
  }
}

/** Constructor.
 * @param host host to connect to
 * @param port port to connect to
 * @param image_id image ID of image to retrieve
 * @param jpeg if true jpeg images will be transferred and automatically be
 * decompressed, otherwise raw images are transferred
 */
NetworkCamera::NetworkCamera(const char *host, unsigned short port, const char *image_id,
			     bool jpeg)
{
  if ( image_id == NULL ) {
    throw NullPointerException("NetworkCamera: image_id must not be NULL");
  }
  if ( host == NULL ) {
    throw NullPointerException("NetworkCamera: host must not be NULL");
  }
  __image_id = strdup(image_id);
  __host = strdup(host);
  __port = port;
  __get_jpeg = jpeg;

  __connected       = false;
  __local_version   = 0;
  __remote_version  = 0;
  __decompressor    = NULL;
  __decompressed_buffer = NULL;
  __last_width = 0;
  __last_height = 0;
  __fuse_image = NULL;
  __fuse_message = NULL;

  __fusec = new FuseClient(__host, __port, this);
  if ( __get_jpeg ) {
    __decompressor = new JpegImageDecompressor();
  }
}


/** Constructor.
 * Initialize with parameters from camera argument parser, supported values are:
 * - host=HOST, hostname or IP of host to connect to
 * - port=PORT, port number to connect to
 * - image=ID, image ID of image to retrieve
 * - jpeg=<true|false>, if true JPEGs are recieved and decompressed otherwise
 *   raw images will be transferred (raw is the default)
 * @param cap camera argument parser
 */
NetworkCamera::NetworkCamera(const CameraArgumentParser *cap)
{
  if ( cap->has("image") ) {
    __image_id = strdup(cap->get("image").c_str());
  } else {
    throw NullPointerException("image parameter must be set");
  }
  if ( cap->has("host") ) {
    __host = strdup(cap->get("host").c_str());
  } else {
    __host = strdup("localhost");
  }
  if ( cap->has("port") ) {
    int i = atoi(cap->get("port").c_str());
    if ( (i < 0) || (i >= 0xFFFF) ) {
      throw IllegalArgumentException("Port must be in the range 0-65535");
    }
    __port = (unsigned int)i;
  } else {
    __port = 2208;
  }

  __get_jpeg = ( cap->has("jpeg") && (cap->get("jpeg") == "true"));

  __connected       = false;
  __local_version   = 0;
  __remote_version  = 0;
  __decompressor    = NULL;
  __decompressed_buffer = NULL;
  __last_width = 0;
  __last_height = 0;
  __fuse_image = NULL;
  __fuse_message = NULL;

  __fusec = new FuseClient(__host, __port, this);
  if ( __get_jpeg ) {
    __decompressor = new JpegImageDecompressor();
  }
}


/** Destructor. */
NetworkCamera::~NetworkCamera()
{
  close();
  delete __fusec;
  free(__host);
  free(__image_id);
  if ( __decompressed_buffer != NULL) free(__decompressed_buffer);
  delete __decompressor;
}


void
NetworkCamera::open()
{
  __fusec->connect();
  __fusec->start();
  __fusec->wait();
  __opened = true;
}


void
NetworkCamera::start()
{
  __started = true;
}

void
NetworkCamera::stop()
{
  __started = false;
}


void
NetworkCamera::print_info()
{
}


void
NetworkCamera::capture()
{
  if (! __connected) {
    throw CaptureException("Capture failed, not connected");
  }
  if ( __fuse_image ) {
    throw CaptureException("You must dispose the buffer before fetching a new image");
  }
  if ( !__image_id ) {
    throw CaptureException("You must specify an image id");
  }

  FUSE_imagereq_message_t *irm = (FUSE_imagereq_message_t *)malloc(sizeof(FUSE_imagereq_message_t));
  memset(irm, 0, sizeof(FUSE_imagereq_message_t));
  strncpy(irm->image_id, __image_id, IMAGE_ID_MAX_LENGTH);
  irm->format = (__get_jpeg ? FUSE_IF_JPEG : FUSE_IF_RAW);
  __fusec->enqueue(FUSE_MT_GET_IMAGE, irm, sizeof(FUSE_imagereq_message_t));

  __fusec->wait();

  if ( ! __fuse_image ) {
    throw CaptureException("Fetching the image failed, no image received");
  }

  if ( __get_jpeg ) {
    if ( (__fuse_image->pixel_width() != __last_width) ||
	 (__fuse_image->pixel_height() != __last_height) ) {
      if (__decompressed_buffer != NULL ) {
	free(__decompressed_buffer);
      }
      size_t buffer_size = colorspace_buffer_size(YUV422_PLANAR, __fuse_image->pixel_width(),
						  __fuse_image->pixel_height());
      __decompressed_buffer = (unsigned char *)malloc(buffer_size);
      __decompressor->set_decompressed_buffer(__decompressed_buffer, buffer_size);
    }
    __decompressor->set_compressed_buffer(__fuse_image->buffer(), __fuse_image->buffer_size());
    __decompressor->decompress();
  }
}


unsigned char *
NetworkCamera::buffer()
{
  if (__get_jpeg) {
    return __decompressed_buffer;
  } else {
    if ( __fuse_image ) {
      return __fuse_image->buffer();
    } else {
    return NULL;
    }
  }
}

unsigned int
NetworkCamera::buffer_size()
{
  if ( __get_jpeg ) {
    return colorspace_buffer_size(YUV422_PLANAR, pixel_width(), pixel_height());
  } else {
    if (! __fuse_image) {
      return 0;
    } else {
      return colorspace_buffer_size((colorspace_t)__fuse_image->colorspace(),
				    __fuse_image->pixel_width(),
				    __fuse_image->pixel_height());
    }
  }
}

void
NetworkCamera::close()
{
  dispose_buffer();
  if ( __started ) {
    stop();
  }
  if ( __opened ) {
    __fusec->disconnect();
    __fusec->cancel();
    __fusec->join();
    __opened = false;
  }
}

void
NetworkCamera::dispose_buffer()
{
  delete __fuse_image;
  __fuse_image = NULL;
  if ( __fuse_message ) {
    __fuse_message->unref();
    __fuse_message = NULL;
  }
}

unsigned int
NetworkCamera::pixel_width()
{
  if ( __fuse_image ) {
    return __fuse_image->pixel_width();
  } else {
    return 0;
  }
}

unsigned int
NetworkCamera::pixel_height()
{
  if ( __fuse_image ) {
    return __fuse_image->pixel_height();
  } else {
    return 0;
  }
}


void
NetworkCamera::flush()
{
  if (! __connected)  return;
  dispose_buffer();
}


bool
NetworkCamera::ready()
{
  return __connected;
}


void
NetworkCamera::set_image_id(const char *image_id)
{
  __image_id = strdup(image_id);
}


void
NetworkCamera::set_image_number(unsigned int n)
{
  // ignored, has to go away anyway
}


colorspace_t
NetworkCamera::colorspace()
{
  if ( __get_jpeg ) {
    return YUV422_PLANAR;
  } else {
    if ( __fuse_image ) {
      return (colorspace_t)__fuse_image->colorspace();
    } else {
      return CS_UNKNOWN;
    }
  }
}


std::vector<FUSE_imageinfo_t>&
NetworkCamera::image_list()
{
  __image_list.clear();

  if (! __connected) {
    throw CaptureException("Capture failed, not connected");
  }

  __fusec->enqueue(FUSE_MT_GET_IMAGE_LIST);
  __fusec->wait();

  return __image_list;
}


void
NetworkCamera::fuse_invalid_server_version(uint32_t local_version,
					   uint32_t remote_version) throw()
{
  __local_version  = local_version;
  __remote_version = remote_version;
}


void
NetworkCamera::fuse_connection_established() throw()
{
  __connected = true;
}


void
NetworkCamera::fuse_inbound_received(FuseNetworkMessage *m) throw()
{
  switch(m->type()) {

  case FUSE_MT_IMAGE:
    try {
      __fuse_image = m->msgc<FuseImageContent>();
      if ( __fuse_image ) {
	__fuse_message = m;
	__fuse_message->ref();
      }
    } catch (Exception &e) {
      __fuse_image = NULL;
      __fuse_message = NULL;
    }
    break;

  case FUSE_MT_IMAGE_LIST:
    try {
      FuseImageListContent* fuse_image_list = m->msgc<FuseImageListContent>();
      if (fuse_image_list ) {
	while ( fuse_image_list->has_next() ) {
	  FUSE_imageinfo_t *iip = fuse_image_list->next();
	  FUSE_imageinfo_t ii;
	  strncpy(ii.image_id, iip->image_id, IMAGE_ID_MAX_LENGTH);
	  ii.colorspace = ntohs(iip->colorspace);
	  ii.width = ntohl(iip->width);
	  ii.height = ntohl(iip->height);
	  ii.buffer_size = ntohl(iip->buffer_size);
	  __image_list.push_back(ii);
	  
	  /*
	  printf("NetworkCamera: id: %s  width: %d  height: %d  cs: %s\n",
		 ii.image_id, ii.width, ii.height, colorspace_to_string((colorspace_t) ii.colorspace));
	  */
	}
      }
    }
    catch (Exception &e) {
    }
    break;

  default:
      break;
  }
}

