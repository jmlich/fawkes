
/***************************************************************************
 *  main.cpp - YUV viewer gui
 *
 *  Created:  Sat Mar 22 16:34:02 2009
 *  Copyright 2009 Christof Rath <c.rath@student.tugraz.at>
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "yuv_viewer_gui.h"

#include <core/exception.h>

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  Glib::thread_init();

  try
  {
    Gtk::Main kit(argc, argv);

#ifdef GLIBMM_EXCEPTIONS_ENABLED
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create_from_file(RESDIR"/guis/yuv_viewer/yuv_viewer.ui");
#else
    Glib::RefPtr<Gtk::BuilderError> error;
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create(RESDIR"/guis/yuv_viewer/yuv_viewer.ui", error);
    if (error.get()) {
      throw fawkes::Exception("Failed to load UI file: %s", error->what().c_str());
    }
#endif

    YuvViewerGtkWindow *window = NULL;
    builder->get_widget_derived("wndMain", window);

    kit.run( *window );

    delete window;
  }
  catch (std::exception const& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  return 0;
}
