/*
 * Copyright (c) 2011, mroe.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef WED_XPLUGINDRAWOBJ_H
#define WED_XPLUGINDRAWOBJ_H

#include "WED_XPluginNode.h"
#include "XPLMScenery.h"

class WED_XPluginMgr;

class WED_XPluginDrawObj : public WED_XPluginNode
{
public:

    WED_XPluginDrawObj(XPLMObjectRef inObjRef,WED_XPluginMgr * inRef);
    virtual ~WED_XPluginDrawObj();

    void           Draw(bool isLit);
    void           SetDrawVertex(bool inDrawVertex){mDrawVertex = inDrawVertex;}

protected:
private:

    bool           mWantDraw;
    bool           mDrawVertex;
    XPLMObjectRef  mObjRef;
};

#endif // WED_XPLUGINDRAWOBJ_H