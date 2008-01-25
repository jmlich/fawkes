
/***************************************************************************
 *  pnm.cpp - PNM reader
 *
 *  Generated: Sun Jan 13 16:23:08 2008
 *  Copyright  2007  Daniel Beck
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

#include <fvutils/readers/pnm.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/conversions.h>
#include <core/exception.h>

#include <cstdlib>
#include <cstring>

/** @class PNMReader <fvutils/readers/pnm.h>
 * PNM file reader.
 *
 * @author Daniel Beck
 */

/** Constructor.
 * @param filename name of the PNM file
 */
PNMReader::PNMReader(const char* filename)
{
  m_filename = strdup(filename);
  m_pnmfile = fopen(m_filename, "rb");

  if ( m_pnmfile == NULL ) 
    {
      throw Exception("PNMReader::ctor: cannot open PNM file");
    }
  
  // read header
  char* line = (char*) malloc(80);
  
  // magic value
  fgets(line, 80, m_pnmfile);

  if ( strcmp("P6", line) > 0 )
    {
      throw Exception("PNMReader::ctor: unknown magic value");
    }

  // comments
  do
    {
      fgets(line, 80, m_pnmfile);
    }
  while ( strncmp("#", line, 1) == 0);
  
  // width & height
  char* tmp = (char*) malloc(10);
  char* token;
  token = strtok(line, " ");
  if ( atoi(token) >= 0 ) { m_img_width = (unsigned int) atoi(token); }
  else { throw Exception("PNMReader::ctor: could not read out image width"); };
  token = strtok(NULL, " ");
  if ( atoi(token) >= 0 ) { m_img_height = (unsigned int) atoi(token); }
  else { throw Exception("PNMReader::ctor: could not read out image height"); };
  free(tmp);

  // depth
  fgets(line, 80, m_pnmfile);
  int max = atoi(line);
  free(line);
  if ( max >= 0) 
    { 
      switch(max)
	{
	case 1:
	  m_img_depth = 1;
	  break;
	  
	case 15:
	  m_img_depth = 2;
	  break;

	case 255:
	  m_img_depth = 3;
	  break;
	  
	default:
	  break;
	}
    }
  else
    {
      throw Exception("PNMReader::ctor: unknown color depth");
    }

  size_t img_size = m_img_width * m_img_height * m_img_depth;
  m_pnm_buffer = (unsigned char*) malloc(img_size);
}

/** Destructor. */
PNMReader::~PNMReader()
{
  free(m_filename);
  free(m_pnm_buffer);
}

void
PNMReader::set_buffer(unsigned char* buffer)
{
  m_yuv_buffer = buffer;
}

colorspace_t
PNMReader::colorspace()
{
  return YUV422_PLANAR;
}

unsigned int
PNMReader::pixel_width()
{
  return m_img_width;
}

unsigned int
PNMReader::pixel_height()
{
  return m_img_height;
}

void
PNMReader::read()
{
  if (m_yuv_buffer == NULL)
    {
      throw Exception("PNMReader::read: buffer = NULL");
    }

  fread(m_pnm_buffer, m_img_depth, m_img_width * m_img_height, m_pnmfile); 
  convert(RGB, YUV422_PLANAR, m_pnm_buffer, m_yuv_buffer, m_img_width, m_img_height);
}