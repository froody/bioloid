/**************************************************************************

    Copyright 2007, 2008 Rainer Jäkel <rainer.jaekel@googlemail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

**************************************************************************/

#ifndef __CONFIGURATION

#define __CONFIGURATION

#include <vector>
#include "tinyxml.h"

/*! \brief Configuration file wrapper

   Loads and stores configuration data (in a xml file). Set \a rootNode
   to root node of xml file first, then use setters and getters to access the data.
   \see CGlobalContainer::init
 */
class CConfiguration
{
private:
  TiXmlDocument doc;

public:
  CConfiguration();
  ~CConfiguration();
  void findNodes(const char*name, std::vector<TiXmlElement*> &result);

  TiXmlElement*findNode(const char*name);

  char rootNode[255];
  void   setRootNode(const char*node);

  bool   load(const char*cfgFile);
  void   save(const char*cfgFile);

  float  getFloat(const char*str, float def = 0.0);
  double getDouble(const char*str, double def = 0.0);
  char*getString(const char*str, const char*def = "");
  int    getInteger(const char*str, int def = 0);
  bool   getBoolean(const char*str, bool def = false);

  void   setFloat(char*str, float value);
  void   setDouble(char*str, double value);
  void   setString(char*str, char*value);
  void   setInteger(char*str, int value);
  void   setBoolean(char*str, bool value);

  static float  getAttributeFloat(TiXmlElement*node, const char*str, float def = 0.0);
  static double getAttributeDouble(TiXmlElement*node, const char*str, double def = 0.0);
  static char*getAttributeString(TiXmlElement*node, const char*str, char*def = "");
  static int    getAttributeInteger(TiXmlElement*node, const char*str, int def = 0);
  static bool   getAttributeBoolean(TiXmlElement*node, const char*str, bool def = false);

  static void   setAttributeFloat(TiXmlElement*node, char*str, float value);
  static void   setAttributeDouble(TiXmlElement*node, char*str, double value);
  static void   setAttributeString(TiXmlElement*node, char*str, char*value);
  static void   setAttributeInteger(TiXmlElement*node, char*str, int value);
  static void   setAttributeBoolean(TiXmlElement*node, char*str, bool value);

};

#endif
