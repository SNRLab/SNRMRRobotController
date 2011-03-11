//====================================================================
//
// MRI guided robot control system
//
// Copyright (C) 2003-2005 by The University of Tokyo,
// All Right Reserved.
// Copyright (C) 2005-2009 by Shiga University of Medical Science,
// All Right Reserved.
// Copyright (C) 2009-2011 by Brigham and Women's Hospital
// All Right Reserved.
//
//====================================================================
// $RCSfile: MrsvrNavigationWindow.cpp,v $
// $Revision: 1.19 $ 
// $Author: junichi $
// $Date: 2006/01/20 03:15:47 $
//====================================================================


#ifndef _INC_MRSVR_NAVIGATION_DIALOG
#define _INC_MRSVR_NAVIGATION_DIALOG

#include <fx.h>


//Class Maier Dialog Box
class FXNavigationDialog : public FXDialogBox {
  FXDECLARE(FXNavigationDialog)
protected:
  FXHorizontalFrame* frplPl;


private:
  FXNavigationDialog(){}
  
  FXDataTarget *dtNewTarget[3];
  float valNewTarget[3];

  FXDataTarget *dtOldTarget[3];
  float valOldTarget[3];

  FXDataTarget *dtDeltaTarget[3];
  float valDeltaTarget[3];

  FXDataTarget *dtTemp[3];
  float valTemp[3];

  //Font for Canvas Text
  FXFont *canvasFont0;
  
public:
  FXNavigationDialog(FXWindow* owner);
  long onCmdClear();
  long onPaintTarget();
public:  
  //Message Handler
  long onPaint(FXObject*,FXSelector,void*);
  long onCmdTimer(FXObject*, FXSelector,void*);
private: 
   FXCanvas *canvas;

public: //Messages
  enum{
   ID_CANVAS2=FXDialogBox::ID_LAST,
   ID_MOVETO_TARGET,
   ID_PAINT_TARGET,
   ID_UPDATE_PARAMETER,
   ID_TIMER,
   ID_LAST
   };

  virtual ~FXNavigationDialog();
};




#endif //  _INC_MRSVR_NAVIGATION_DIALOG

