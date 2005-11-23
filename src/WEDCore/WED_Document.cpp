#include "WED_Document.h"
#include "WED_Progress.h"
#include "XESIO.h"
#include "AptIO.h"
#include "MapAlgs.h"
// TODO: 
// migrate all old stuff
// wire dirty to obj persistence

WED_Document::WED_Document(const string& path) : 
	mArchive(),
	mUndo(&mArchive),
	mFilePath(path),
	mDirty(false)
{
}

WED_Document::~WED_Document()
{
}

string				WED_Document::GetFilePath(void) const
{
	return mFilePath;
}

bool				WED_Document::GetDirty(void) const
{
	return mDirty;
}

void				WED_Document::Save(void)
{
	WriteXESFile(mFilePath.c_str(), gMap, gTriangulationHi, gDem, gApts, WED_ProgressFunc);
	mDirty = false;
}

void				WED_Document::Load(void)
{
	MFMemFile *	memFile = MemFile_Open(mFilePath.c_str());
	if (memFile)
	{
		gMap.clear();
		gDem.clear();
		mDirty = false;
		gTriangulationHi.clear();
//		gPointFeatureSelection.clear();

		ReadXESFile(memFile, &gMap, &gTriangulationHi, &gDem, &gApts, WED_ProgressFunc);
		IndexAirports(gApts, gAptIndex);
		MemFile_Close(memFile);
	} else
		return;

	Point2	sw, ne;
	CalcBoundingBox(gMap, sw, ne);
	double mapNorth = (ne.y);
	double mapSouth = (sw.y);
	double mapEast = (ne.x);
	double mapWest = (sw.x);

	if (!gMap.is_valid())
	{
		gMap.clear();
		mDirty = false;
	}	
	
	mDirty = false;
}

void				WED_Document::SaveAs(const string& inName, bool migrate)
{
	WriteXESFile(inName.c_str(), gMap, gTriangulationHi, gDem, gApts, WED_ProgressFunc);
	mDirty = false;
	if (migrate) mFilePath = inName;
	
}

