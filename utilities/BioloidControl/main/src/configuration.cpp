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


#include <cstdio>
#include <iostream>
#include <cstring>
#include <fstream>

using namespace std;

#include "../include/configuration.h"

CConfiguration::CConfiguration() {
}

CConfiguration::~CConfiguration() {
}

void CConfiguration::setRootNode(const char*node) {
  if(node != NULL) {
    sprintf(rootNode, node);
  }
  else {
    sprintf(rootNode, "");
  }
}

bool CConfiguration::load(const char*cfgFile) {
  bool okay = doc.LoadFile(cfgFile);
  return okay;
}

void CConfiguration::save(const char*cfgFile) {
}

TiXmlElement*CConfiguration::findNode(const char*name) {
  TiXmlElement*iter = doc.FirstChildElement(rootNode);

  char *pch;
  char sBuffer[255];
  strcpy(sBuffer, name);

  pch = strtok(sBuffer, ".");

  while(iter && (pch != NULL) ) {
    iter = iter->FirstChildElement(pch);
    pch = strtok(NULL, ".");
  }

  if( (pch == NULL) && (iter) ) {
    return iter;
  }
  else {
    return NULL;
  }
}

void tokenize(const std::string&str,
              std::vector<std::string>&tokens,
              const std::string&delimiters)
{
  // Skip delimiters at beginning.
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
  // Find first "non-delimiter".
  std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

  while(std::string::npos != pos || std::string::npos != lastPos) {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(lastPos, pos - lastPos) );
    // Skip delimiters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimiters, pos);
    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, lastPos);
  }
}

void CConfiguration::findNodes(const char*name, std::vector<TiXmlElement*> &result) {
  TiXmlElement*iter = doc.FirstChildElement(rootNode);

  std::string str = name;
  std::vector<std::string> tokens;
  tokenize(str, tokens, ".");

  int index = 0;
  while(iter != NULL && index < tokens.size() ) {
    iter = iter->FirstChildElement(tokens[index].c_str() );
    index++;
  }

  if(iter == NULL) {
    return;
  }

  while(iter != NULL) {
    result.push_back(iter);
    iter = iter->NextSiblingElement(tokens[tokens.size()-1].c_str() );
  }
}

float CConfiguration::getAttributeFloat(TiXmlElement*node, const char*str, float def) {
  const char*value = getAttributeString(node, str, (char *)"");
  if(strcmp(value, "") == 0) {
    return def;
  }
  else { return atof(value);}
}

double CConfiguration::getAttributeDouble(TiXmlElement*node, const char*str, double def) {
  const char*value = getAttributeString(node, str, (char *)"");
  if(strcmp(value, "") == 0) {
    return def;
  }
  else { return atof(value);}
}

char*CConfiguration::getAttributeString(TiXmlElement*node, const char*str, char*def) {
  char*tmpstr = (char*)node->Attribute( (char *)str);
  if(tmpstr == NULL) {
    return def;
  }
  else { return tmpstr;}
}

int CConfiguration::getAttributeInteger(TiXmlElement*node, const char*str, int def) {
  const char*value = getAttributeString(node, str, (char *)"");
  if(strcmp(value, "") == 0) {
    return def;
  }
  else { return atoi(value);}
}

bool CConfiguration::getAttributeBoolean(TiXmlElement*node, const char*str, bool def) {
  const char*value = getAttributeString(node, str, (char *)"");
  if(strcmp(value, "") == 0) {
    return def;
  }
  else if(strcasecmp(value, "true") == 0) {
    return true;
  }
  else if(strcasecmp(value, "false") == 0) {
    return false;
  }
  else { return def;}
}

void CConfiguration::setAttributeFloat(TiXmlElement*node, char*str, float value) {
  char buffer[255];
  sprintf(buffer, "%f", value);
  node->SetAttribute(str, buffer);
}

void CConfiguration::setAttributeDouble(TiXmlElement*node, char*str, double value) {
  char buffer[255];
  sprintf(buffer, "%f", value);
  node->SetAttribute(str, buffer);
}

void CConfiguration::setAttributeString(TiXmlElement*node, char*str, char*value) {
  node->SetAttribute(str, value);
}

void CConfiguration::setAttributeInteger(TiXmlElement*node, char*str, int value) {
  char buffer[255];
  sprintf(buffer, "%d", value);
  node->SetAttribute(str, buffer);
}

void CConfiguration::setAttributeBoolean(TiXmlElement*node, char*str, bool value) {
  if(value) {
    node->SetAttribute(str, (char *)"true");
  }
  else {
    node->SetAttribute(str, (char *)"false");
  }
}

void CConfiguration::setFloat(char*str, float value) {
  char buffer[255];
  sprintf(buffer, "%f", value);
  setString(str, buffer);
}

void CConfiguration::setDouble(char*str, double value) {
  char buffer[255];
  sprintf(buffer, "%f", value);
  setString(str, buffer);
}

void CConfiguration::setString(char*str, char*value) {
  TiXmlElement*result = findNode(str);

  if(result != NULL) {
    TiXmlText*text = result->FirstChild()->ToText();
    text->SetValue(value);
  }
}

void CConfiguration::setInteger(char*str, int value) {
  char buffer[255];
  sprintf(buffer, "%d", value);
  setString(str, buffer);
}

void CConfiguration::setBoolean(char*str, bool value) {
  if(value) {
    setString(str, (char *)"true");
  }
  else {
    setString(str, (char *)"false");
  }
}

char*CConfiguration::getString(const char*str, const char*def) {
  TiXmlElement*result = findNode(str);

  if(result != NULL) {
    return (char*)result->GetText();
  }
  else {
    return (char *)def;
  }
}

float CConfiguration::getFloat(const char*str, float def) {
  return (float)getDouble(str, (double)def);
}

double CConfiguration::getDouble(const char*str, double def) {
  const char*value = getString(str, "");
  if(strcmp(value, "") == 0) {
    return def;
  }
  else { return atof(value);}
}

int CConfiguration::getInteger(const char*str, int def) {
  const char*value = getString(str, "");
  if(strcmp(value, "") == 0) {
    return def;
  }
  else { return atoi(value);}
}

bool CConfiguration::getBoolean(const char*str, bool def) {
  const char*value = getString(str, "");
  if(strcmp(value, "") == 0) {
    return def;
  }
  else if(strcasecmp(value, "true") == 0) {
    return true;
  }
  else if(strcasecmp(value, "false") == 0) {
    return false;
  }
  else { return def;}
}

