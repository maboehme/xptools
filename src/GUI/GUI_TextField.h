#ifndef GUI_TEXTFIELD_H
#define GUI_TEXTFIELD_H

#include "OGLE.h"
#include "GUI_Pane.h"
#include "GUI_ScrollerPane.h"
#include "GUI_Commander.h"

class	GUI_GraphState;

class	GUI_TextField : public GUI_Pane, 
						public GUI_Commander,
						public GUI_ScrollerPaneContent,
						public OGLE {
public:

						 GUI_TextField(int h_scroll, GUI_Commander * parent);
	virtual				~GUI_TextField();

			void		SetWidth(float width);

	// GUI_Pane
	virtual	void		Draw(GUI_GraphState * state);
	virtual	int			MouseDown(int x, int y, int button);
	virtual	void		MouseDrag(int x, int y, int button);
	virtual	void		MouseUp(int x, int y, int button);
	virtual	int			ScrollWheel(int x, int y, int dist, int axis);
	virtual void		SetBounds(int x1, int y1, int x2, int y2);
	virtual void		SetBounds(int inBounds[4]);

	// GUI_Commander
	virtual	int			KeyPress(char inKey, int inVK, GUI_KeyFlags inFlags);
	virtual	int			HandleCommand(int command);
	virtual	int			CanHandleCommand(int command, string& ioName, int& ioCheck);
	virtual	int			AcceptTakeFocus(void);
	virtual int			AcceptLoseFocus(int inForce);
	virtual	int			AcceptFocusChain(void);

	// GUI_ScrollerPaneContent
	virtual	void		GetScrollBounds(float outTotalBounds[4], float outVisibleBounds[4]);
	virtual	void		ScrollH(float xOffset);
	virtual	void		ScrollV(float yOffset);

protected:

	// OGLE
	virtual	void			GetVisibleBounds(
								float			bounds[4]);
	virtual	void			GetLogicalBounds(
								float			bounds[4]);
	virtual	void			SetLogicalHeight(
								float 			height);
	virtual	void			ScrollTo(
								float			where[4]);
	virtual	void			GetText(
								const char **	start_p,
								const char **	end_p);
	virtual	void			ReplaceText(
								int				offset1,
								int				offset2,
								const char *	t1,
								const char *	t2);
	virtual	float			GetLineHeight(void);
//	virtual	float			GetBaseline(void);
	virtual	float			MeasureString(
								const char * 	tStart, 
								const char * 	tEnd);
	virtual	int				FitStringFwd(
								const char * 	tStart, 
								const char * 	tEnd, 
								float 			space);
	virtual	int				FitStringRev(
								const char * 	tStart, 
								const char * 	tEnd, 
								float 			space);
	virtual	void			DrawString(
								const char *	tStart,
								const char *	tEnd,
								float			x,
								float			y);
	virtual	void			DrawSelection(	
								float			bounds[4]);
	virtual	const char *	WordBreak(
								const char *	t1,
								const char *	t2);
	
private:

		int					mScrollH;
		float				mLogicalBounds[4];
		GUI_GraphState * 	mState;
		string				mText;
};

#endif /* GUI_TEXTFIELD_H */

