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


#ifndef __LIST

#define __LIST

// mdda - this is pretty redundant - only CListIterator is used

/* ! \brief Iterator base class
 */

/*
   class CIterator
   {
      public:

      virtual void* next();
      virtual bool  hasNext();
   };
 */

/*! \brief Linked list item
 */
class CListItem
{
public:
  void*data;
  CListItem*next;
  CListItem*prev;
};

// mdda : doing this virtualization thing only slows us down
//class CListIterator: public CIterator

/*! \brief List Iterator
 */
class CListIterator
{
private:
  CListItem*base;
  bool justStarted;

public:
  CListIterator(CListItem *base);

  void*next();
  bool  hasNext();

};

/*! \brief Double linked list

   Simple double linked list.
 */
class CList
{
private:
  int length;
  CListItem*first;
  CListItem*last;

public:
  CList();

  void add(void*data);
  int  count();
  void clear(bool freeData = true);
  void*removeFirst(bool freeData = true);
  void*removeLast(bool freeData = true);
  CListItem*getFirst();
  CListItem*getLast();
  CListIterator getIterator();

};

#endif
