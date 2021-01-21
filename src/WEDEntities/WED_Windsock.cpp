/*
 * Copyright (c) 2007, Laminar Research.
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

#include "WED_Windsock.h"
#include "AptDefs.h"
#include "XESConstants.h"

DEFINE_PERSISTENT(WED_Windsock)
TRIVIAL_COPY(WED_Windsock, WED_GISPoint)

WED_Windsock::WED_Windsock(WED_Archive * a, int i) : WED_GISPoint(a,i),
	lit(this,PROP_Name("Lit", XML_Name("windsock","lit")),0)
{
}

WED_Windsock::~WED_Windsock()
{
}

void	WED_Windsock::SetLit(int l)
{
	lit = l;
}

void	WED_Windsock::Import(const AptWindsock_t& x, void (* print_func)(void *, const char *, ...), void * ref)
{
	SetLocation(gis_Geo,x.location);
	SetName(x.name);
	lit = x.lit;
}

void	WED_Windsock::Export(		 AptWindsock_t& x) const
{
	GetLocation(gis_Geo,x.location);
	GetName(x.name);
	x.lit = lit;
}

void	WED_Windsock::GetBounds(GISLayer_t l, Bbox2&  bounds) const
{
	constexpr double MAX_RADIUS = 5.0;

	WED_GISPoint::GetBounds(l, bounds);
	double mtr_to_lon = MTR_TO_DEG_LAT / cos(bounds.ymin() * DEG_TO_RAD);
	bounds.expand(mtr_to_lon * MAX_RADIUS);
}

Bbox3	WED_Windsock::GetVisibleBounds() const
{
	constexpr double MAX_HEIGHT = 8.0;

	Bbox2 bb2;
	GetBounds(gis_Geo, bb2);
	Bbox3 bb3(bb2);
	bb3.p2.z = MAX_HEIGHT;
	return bb3;
}