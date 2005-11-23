#ifndef WED_DOCUMENT_H
#define WED_DOCUMENT_H

#include "WED_Archive.h"
#include "WED_UndoMgr.h"
#include "WED_Globals.h"
#include "MeshDefs.h"
#include "AptDefs.h"
#include "MapDefs.h"
#include "DEMDefs.h"


class	WED_Document {
public:

						WED_Document(const string& path);
						~WED_Document();

	// Management
	string				GetFilePath(void) const;
	bool				GetDirty(void) const;
	
	void				Save(void);
	void				Load(void);
	void				SaveAs(const string& inName, bool migrate);

	// OBJECT PLACEMENT
	
	// LEGACY STUFF
	
	Pmwx				gMap;
	DEMGeoMap			gDem;
	CDT					gTriangulationHi;
	AptVector			gApts;
	AptIndex			gAptIndex;

private:

	WED_Archive		mArchive;
	WED_UndoMgr		mUndo;


	string				mFilePath;
	bool				mDirty;

	WED_Document();
	WED_Document(const WED_Document&);
	WED_Document& operator=(const WED_Document&);

};

#endif