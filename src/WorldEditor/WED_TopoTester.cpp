#include "WED_TopoTester.h"
#include "MapDefs.h"
#include "XPLMGraphics.h"
#include "WED_MapZoomer.h"
#include "WED_DrawMap.h"
#include "WED_Globals.h"
#include "WED_Document.h"
#include "WED_Selection.h"
#include "WED_Notify.h"
#include "WED_Msgs.h"

WED_TopoTester::WED_TopoTester(WED_MapZoomer * inZoomer) : WED_MapTool(inZoomer), mRayShoot(false)
{
}
WED_TopoTester::~WED_TopoTester()
{
}

void	WED_TopoTester::DrawFeedbackUnderlay(
				bool				inCurrent)
{
}

void	WED_TopoTester::DrawFeedbackOverlay(
				bool				inCurrent)
{
	int mx, my;
	XPLMGetMouseLocation(&mx, &my);
	
	vector<Point2>		fps;
	if (inCurrent && mRayShoot)
	{
		mTarget.x = GetZoomer()->XPixelToLon(mx);
		mTarget.y = GetZoomer()->YPixelToLat(my);
		
		Vector2	dist(mAnchor, mTarget);
		if (XPLMGetModifiers() & xplm_ShiftFlag)
		{
			if (abs(dist.dx) < abs(dist.dy))
				mTarget.x = mAnchor.x;
			else
				mTarget.y = mAnchor.y;
		}

		gVertexSelection.clear();						
		gFaceSelection.clear();						
		gEdgeSelection.clear();						
	
		Point2				st_p = mAnchor;
		GISHalfedge *		st_h = mAnchorHint;
		Pmwx::Locate_type	st_l = mAnchorLoc;

		fps.push_back(mAnchor);
		
		while (st_p != mTarget)
		{
			mFoundHint = gDocument->gMap.ray_shoot(st_p, st_l, st_h,
									mTarget, mFound, mFoundLoc);
			
			
			if (mFoundHint != NULL)
			switch(mFoundLoc) {
			case Pmwx::locate_Face:	
				if (mFoundHint->face() != gDocument->gMap.unbounded_face())
					gFaceSelection.insert(mFoundHint->face());
				gSelectionMode = wed_Select_Face;
				break;
			case Pmwx::locate_Halfedge:	
				gEdgeSelection.insert(mFoundHint->mDominant ? mFoundHint : mFoundHint->twin());
				gSelectionMode = wed_Select_Edge;
				break;
			case Pmwx::locate_Vertex:	
				gVertexSelection.insert(mFoundHint->target());
				gSelectionMode = wed_Select_Vertex;
				break;
			}
			st_p = mFound;
			st_l = mFoundLoc;
			st_h = mFoundHint;
			fps.push_back(mFound);
		}
	}
	if (inCurrent && !mRayShoot)
	{
		mAnchor.x = GetZoomer()->XPixelToLon(mx);
		mAnchor.y = GetZoomer()->YPixelToLat(my);
		mAnchorHint = gDocument->gMap.locate_point(mAnchor, mAnchorLoc);

		gVertexSelection.clear();						
		gFaceSelection.clear();						
		gEdgeSelection.clear();						

		if (mAnchorHint != NULL)
		switch(mAnchorLoc) {
		case Pmwx::locate_Face:	
			if (mAnchorHint->face() != gDocument->gMap.unbounded_face())
				gFaceSelection.insert(mAnchorHint->face());
			gSelectionMode = wed_Select_Face;
			break;
		case Pmwx::locate_Halfedge:	
			gEdgeSelection.insert(mAnchorHint->mDominant ? mAnchorHint : mAnchorHint->twin());
			gSelectionMode = wed_Select_Edge;
			break;
		case Pmwx::locate_Vertex:
			gVertexSelection.insert(mAnchorHint->target());
			gSelectionMode = wed_Select_Vertex;
			break;
		}
		
	}

	if (inCurrent && mRayShoot)
	{
		XPLMSetGraphicsState(0, 0, 0, 1, 1, 0, 0);
		glColor4f(1.0, 0.0, 1.0, 0.8);
		glBegin(GL_LINE_STRIP);
		for (int n = 0; n < fps.size(); ++n)
			glVertex2f( GetZoomer()->LonToXPixel(fps[n].x),
						GetZoomer()->LatToYPixel(fps[n].y));
		glEnd();
		
		glPointSize(3);

		glColor4f(1.0, 1.0, 0.0, 0.8);
		glBegin(GL_POINTS);
		for (int n = 0; n < fps.size(); ++n)
			glVertex2f( GetZoomer()->LonToXPixel(fps[n].x),
						GetZoomer()->LatToYPixel(fps[n].y));
		glEnd();

		glPointSize(1);
	}
}

bool	WED_TopoTester::HandleClick(
				XPLMMouseStatus		inStatus,
				int 				inX, 
				int 				inY, 
				int 				inButton)
{
	if (inButton > 0) return false;
	switch(inStatus) {
	case xplm_MouseDown: 
		mRayShoot = true;  
		break;
	case xplm_MouseUp:   
		if (XPLMGetModifiers() & xplm_OptionAltFlag)
		{
			gDocument->gMap.insert_edge(mAnchor, mTarget, NULL, NULL);
			WED_Notifiable::Notify(wed_Cat_File, wed_Msg_VectorChange, NULL);
		}
		mRayShoot = false; 
		break;
	}
	return 1;
}

int		WED_TopoTester::GetNumProperties(void) { return 0; }
void	WED_TopoTester::GetNthPropertyName(int, string&) { }
double	WED_TopoTester::GetNthPropertyValue(int) { return 0.0; }
void	WED_TopoTester::SetNthPropertyValue(int, double) { }

int		WED_TopoTester::GetNumButtons(void) { return 0; }
void	WED_TopoTester::GetNthButtonName(int, string&) { }
void	WED_TopoTester::NthButtonPressed(int) { }

char *	WED_TopoTester::GetStatusText(void)
{
	if (mRayShoot)
	{
		switch(mFoundLoc) {
		case Pmwx::locate_Face:				return "Ray Face";
		case Pmwx::locate_Vertex:			return "Ray Vertex";
		case Pmwx::locate_Halfedge:			return "Ray Halfedge";
		}
	} else {
		switch(mAnchorLoc) {
		case Pmwx::locate_Face:				return "Face";
		case Pmwx::locate_Vertex:			return "Vertex";
		case Pmwx::locate_Halfedge:			return "Halfedge";
		}
	}
	
	return NULL;
}
