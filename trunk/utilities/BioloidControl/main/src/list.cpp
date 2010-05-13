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

// very old, don't take it serious
// mdda : but it is used in (a) the motion code and the (b) physics...

#include <stdlib.h>
#include "../include/list.h"
#include "../include/util.h"

#define DOFREE

// implementation of the iterator class
// mdda - but it is not used - simplify...

/*
   void* CIterator::next()
   {
      return NULL;
   }
   bool  CIterator::hasNext()
   {
      return false;
   }
 */

// linked list iterator class
CListIterator::CListIterator(CListItem *base)
{
  this->base = base;
  justStarted = true;
}

// returns next data element
void*CListIterator::CListIterator::next()
{
  if(base == NULL) {
    return NULL;
  }

  if(justStarted) {
    justStarted = false;
  }
  else {
    base = base->next;
  }

  if(base == NULL) {
    return NULL;
  }

  return base->data;
}

// are data elements left?
bool CListIterator::CListIterator::hasNext()
{
  return (justStarted && (base != NULL) ) || (base != NULL && base->next != NULL);
}

// implementation of simple linear list
CList::CList()
{
  length = 0;
  first = NULL;
  last = NULL;
}

// return amount of items stored
int CList::count()
{
  return length;
}

// add item to end of list
void CList::add(void*data)
{
  CListItem *item = new CListItem();
  if(item == NULL) {
    // mdda : This is bad I know, but I don't want CUtil.cpp, since it has a ton of matrix stuff included
#ifndef PHYSICS
    CUtil::cout("CList::add: malloc failed().\n", TEXT_ERROR);
#else
//                                                std::cout << "CList::add: malloc failed().\n";

#endif
    return;
  }

  item->data = data;
  item->prev = last;
  item->next = NULL;

  if(last == NULL) {
    first = last = item;
  }
  else {
    last->next = item;

    last = item;
  }
  length++;
}

// returns list iterator
CListIterator CList::getIterator()
{
  return CListIterator(first);
}

// clears the list
void CList::clear(bool freeData)
{
  CListItem *item = first;
  CListItem *tmp;

  while(item != NULL)
  {
    tmp = item;
    item = item->next;
#ifdef DOFREE
    if(freeData) {
      free(tmp->data);
    }
#endif
    free(tmp);
  }

  first = last = NULL;
  length = 0;
}

// returns first element of list
CListItem*CList::getFirst()
{
  return first;
}

// returns last element of list
CListItem*CList::getLast()
{
  return last;
}

// removes last element of list
void*CList::removeLast(bool freeData)
{
  if(first == last) {
    return removeFirst(freeData);
  }

  CListItem *tmp = last;
  CListItem *iter = first;

  while(iter->next != last)
  {
    iter = iter->next;
  }

  // iter->next == last
  iter->next = NULL;
  last = iter;

  void*result;
#ifdef DOFREE
  if(freeData) {
    free(tmp->data);
    tmp->data = NULL;
  }
#endif
  result = tmp->data;

  free(tmp);
  length--;
  if(length <= 0) {
    length = 0;
  }

  return result;
}

// removes first element of list
void*CList::removeFirst(bool freeData)
{
  if(first == NULL) {
    return NULL;
  }

  CListItem *tmp = first;
  void*result;

#ifdef DOFREE
  if(freeData) {
    free(tmp->data);
    tmp->data = NULL;
  }
#endif
  result = tmp->data;

  if(first == last) {
    last = first->next;
  }

  first = first->next;

  free(tmp);
  length--;
  if(length <= 0) {
    length = 0;
  }

  return result;
}

