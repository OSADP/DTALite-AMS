// Page_Node_Movement.cpp : implementation file
//
//  Portions Copyright 2010 Xuesong Zhou (xzhou99@gmail.com) 

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html


//    This file is part of NeXTA Version 3 (Open-source).

//    NEXTA is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    NEXTA is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with NEXTA.  If not, see <http://www.gnu.org/licenses/>.

#include "stdafx.h"
#include "TLite.h"
#include "Page_Node_Phase.h"

#include "CGridListCtrlEx\CGridColumnTraitEdit.h"
#include "CGridListCtrlEx\CGridColumnTraitCombo.h"
#include "CGridListCtrlEx\CGridRowTraitXP.h"
#include "MainFrm.h"
#include <string>
#include <sstream>
// CPage_Node_Phase dialog

extern CPen g_PenProhibitedMovement;

IMPLEMENT_DYNAMIC(CPage_Node_Phase, CPropertyPage)

CPage_Node_Phase::CPage_Node_Phase()
: CPropertyPage(CPage_Node_Phase::IDD)
, m_bPhasingDataEditMode(FALSE)
, m_CurrentNode_Name(_T(""))
, m_MovementMsg(_T(""))
, m_CycleLength(0)
, m_Offset(0)
, m_bHideRightTurnMovement(TRUE)
, m_bMultiPhaseDisplay(FALSE)
, m_bOptimizationMethod(FALSE)
, m_bUsingObsHourlyCount(FALSE)
{
	m_bColumnWidthIncludeHeader = true;
	m_SelectedMovementIndex = -1;
	m_bModifiedFlag = false;
	m_PeakHourFactor = 1.0;

	m_SelectedTimingPlanNo = 0;

	m_SelectedPhaseNumber = 0;

}

CPage_Node_Phase::~CPage_Node_Phase()
{
}

void CPage_Node_Phase::DoDataExchange(CDataExchange* pDX)
{
	CPropertyPage::DoDataExchange(pDX);

	DDX_Control(pDX, IDC_GRID_Phasing, m_PhasingGrid);
	DDX_Check(pDX, IDC_EDIT_MODE, m_bPhasingDataEditMode);
	DDX_Text(pDX, IDC_EDIT_CURRENT_NODEID, m_CurrentNodeNumber);
	DDX_Text(pDX, IDC_EDIT_CURRENT_NODENAME, m_CurrentNode_Name);
	DDX_Text(pDX, IDC_MOVEMENT_MSG, m_MovementMsg);
	DDX_Control(pDX, IDC_COMBO1, m_ControlTypeComboBox);
	DDX_Control(pDX, IDC_COMBO2, m_RingTypeComboBox);
	DDX_Text(pDX, IDC_EDIT3, m_CycleLength);
	DDX_Text(pDX, IDC_EDIT9, m_Offset);
	DDX_Check(pDX, IDC_EDIT_MODE2, m_bHideRightTurnMovement);
	DDX_Check(pDX, IDC_CHECK_MULTIPLE_PHASE_DIAGRAM, m_bMultiPhaseDisplay);
	DDX_Check(pDX, IDC_CHECK_OPTIMIZATIONMETHOD, m_bOptimizationMethod);
	DDX_Check(pDX, IDC_CHECK_OPTIMIZATION_OBS, m_bUsingObsHourlyCount);
}


BEGIN_MESSAGE_MAP(CPage_Node_Phase, CPropertyPage)
	ON_WM_LBUTTONDOWN()
	ON_WM_PAINT()
	ON_LBN_SELCHANGE(IDC_LIST1, &CPage_Node_Phase::OnLbnSelchangeList1)
	ON_LBN_DBLCLK(IDC_LIST1, &CPage_Node_Phase::OnLbnDblclkList1)
	ON_NOTIFY(GVN_SELCHANGED, IDC_GRID_Phasing, OnGridEndSelChange)

	ON_BN_CLICKED(IDC_EDIT_MODE, &CPage_Node_Phase::OnBnClickedEditMode)
	ON_WM_LBUTTONDBLCLK()
	ON_CBN_SELCHANGE(IDC_COMBO_Timing_Plan, &CPage_Node_Phase::OnCbnSelchangeComboTimingPlan)
	ON_CBN_SELCHANGE(IDC_COMBO1, &CPage_Node_Phase::OnCbnSelchangeCombo1)
	ON_CBN_SELCHANGE(IDC_COMBO2, &CPage_Node_Phase::OnCbnSelchangeCombo2)
	ON_BN_CLICKED(IDC_EDIT_MODE2, &CPage_Node_Phase::OnBnClickedEditMode2)
	ON_BN_CLICKED(IDC_BUTTON_QEM, &CPage_Node_Phase::OnBnClickedButtonQem)
	ON_BN_CLICKED(IDC_CHECK_MULTIPLE_PHASE_DIAGRAM, &CPage_Node_Phase::OnBnClickedCheckMultiplePhaseDiagram)
	ON_BN_CLICKED(IDC_CHECK_OPTIMIZATIONMETHOD, &CPage_Node_Phase::OnBnClickedCheckOptimizationmethod)
	ON_BN_CLICKED(IDC_CHECK_OPTIMIZATION_OBS, &CPage_Node_Phase::OnBnClickedCheckOptimizationObs)
END_MESSAGE_MAP()



BOOL CPage_Node_Phase::OnInitDialog()
{

	CPropertyPage::OnInitDialog();

	for(int p = 1; p <= 	_max_phase_number; p++)
	{
		m_bAvailablePhaseVector [p] = false; 
		m_EffectiveGreenTime [p]= 0;
	}

	m_CurrentNodeID =  m_pDoc->m_SelectedNodeID ;
	m_CurrentNodeNumber = m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_NodeNumber ;


	// Give better margin to editors


	CString str;
	str.Format("Unknown Control (%d)",m_pDoc->m_ControlType_UnknownControl);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_UnknownControl);

	str.Format("No Control (%d)",m_pDoc->m_ControlType_NoControl);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_NoControl);

	str.Format("Yield Sign (%d)",m_pDoc->m_ControlType_YieldSign);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_NoControl);

	str.Format("2 Way Stop Sign (%d)",m_pDoc->m_ControlType_2wayStopSign);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_2wayStopSign);

	str.Format("4 Way Stop Sign (%d)",m_pDoc->m_ControlType_4wayStopSign);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_4wayStopSign);

	str.Format("Pretimed Signal (%d)",m_pDoc->m_ControlType_PretimedSignal);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_PretimedSignal);

	str.Format("Actuated Signal  (%d)",m_pDoc->m_ControlType_ActuatedSignal);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_ActuatedSignal);

	str.Format("Roundabout  (%d)",m_pDoc->m_ControlType_Roundabout);
	m_ControlTypeComboBox.AddString (str);
	m_ControlTypeVector.push_back(m_pDoc->m_ControlType_Roundabout);


	DTA_Phasing_Data_Matrix element = m_pDoc->GetPhaseData(m_CurrentNodeNumber ,  m_pDoc->m_TimingPlanVector[m_SelectedTimingPlanNo].timing_plan_name);

	m_CycleLength = atoi(element.GetString (DTA_SIG_PHASE_VALUE, TIMING_CycleLength));

	m_Offset = atoi(element.GetString (DTA_SIG_PHASE_VALUE, TIMING_Offset));

	UpdateData(0);

	for(unsigned int i=0; i< m_ControlTypeVector.size(); i++)
	{
		if(m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_ControlType == m_ControlTypeVector[i])
			m_ControlTypeComboBox.SetCurSel (i);
	}

	if(m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_ControlType != m_pDoc->m_ControlType_PretimedSignal
		&& m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_ControlType != m_pDoc->m_ControlType_ActuatedSignal)
	{

		return true;
	}


	m_RingTypeComboBox.AddString ("single_ring");
	m_RingTypeComboBox.AddString ("dual_ring");


	CString ring_type = element.GetString (DTA_SIG_PHASE_VALUE, TIMING_RingType);

	if(ring_type.Find("Single") != -1 || ring_type.Find("single") != -1)
	{
		m_RingTypeComboBox.SetCurSel (0);
	}else
	{
		m_RingTypeComboBox.SetCurSel (1);

	}

	CString optimization_method = element.GetString (DTA_SIG_PHASE_VALUE, TIMING_OptimizationMethod);


	if(optimization_method.Compare ("no") ==0)
	{
		m_bOptimizationMethod = false;
		
		GetDlgItem(IDC_BUTTON_QEM) ->EnableWindow(0);
	
	}
	else
		m_bOptimizationMethod = true;



	m_PhasingGrid.SetDoubleBuffering(1);


	for(unsigned int i = 0; i < m_pDoc->m_TimingPlanVector.size(); i++)
	{

		DTA_Phasing_Data_Matrix element_current = m_pDoc->GetPhaseData(m_CurrentNodeNumber ,  m_pDoc->m_TimingPlanVector[i].timing_plan_name);

		CString VolumeAdjustmentFactor = element_current.GetString (DTA_SIG_PHASE_VALUE, TIMING_VolumeAdjustmentFactor);

		if(VolumeAdjustmentFactor.GetLength () ==0)
			VolumeAdjustmentFactor = "1";

		CString TimingPlanStr;
		TimingPlanStr.Format ("Plan %d: %s", i+1, m_pDoc->m_TimingPlanVector[i].timing_plan_name.c_str ());

		if(i==0)
		{
		m_VolumeAdjustmentFactor = atof(VolumeAdjustmentFactor);
		
		}

	}



	UpdatePhaseData();
	UpdateData(0);
	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}


void CPage_Node_Phase::UpdatePhaseData()
{


	DisplayPhasingGrid();

}

void CPage_Node_Phase::UpdateList()
{

}

void CPage_Node_Phase::DisplayList()
{


}
void CPage_Node_Phase::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	CRect PlotRect;
	GetClientRect(PlotRect);
	m_PlotRect = PlotRect;

	m_PlotRect.top += 35;
	m_PlotRect.bottom -= 250;
	m_PlotRect.left += 50;
	m_PlotRect.right -= 50;

	int GreenTimeDiagramHeight = 25;
	if(m_bMultiPhaseDisplay == false)
		DrawMovements(&dc,m_PlotRect,false);

	if(m_bMultiPhaseDisplay == true)
	{
		int number_of_rectangles = 8;

		int width = m_PlotRect.Width ()/4;
		int height = (m_PlotRect.Height  ()- 2*GreenTimeDiagramHeight)/2;

		CRect OriginalPlotRect = m_PlotRect;
		CRect GreenTimePlotRect = m_PlotRect;

		GreenTimePlotRect.left  +=5;
		GreenTimePlotRect.right   -=5;

		GreenTimePlotRect.top = m_PlotRect.bottom - 2*GreenTimeDiagramHeight;
		GreenTimePlotRect.bottom  = m_PlotRect.bottom - 1*GreenTimeDiagramHeight;

		CRect GreenTimePlotRect2 = GreenTimePlotRect;


		GreenTimePlotRect2.top = GreenTimePlotRect.bottom +5;
		GreenTimePlotRect2.bottom  = GreenTimePlotRect2.top+ GreenTimeDiagramHeight;


		CBrush  RedBrush(RGB(255,0,0)); 

		dc.SelectObject(&RedBrush);

		dc.Rectangle (GreenTimePlotRect);

		if(m_RingTypeComboBox.GetCurSel () ==1)  //dual ring
		{
			dc.Rectangle (GreenTimePlotRect2);
		}

		for(int p = 1; p <= number_of_rectangles; p++)
		{
			int column = (p-1)%4 ;
			//		int column = p-1;
			int row  = 0;

			if(p>=5)
				row  = 1;

			CRect PhaseRect;

			PhaseRect.left = OriginalPlotRect.left + column* width;
			PhaseRect.right  = OriginalPlotRect.left + (column+1)* width;

			PhaseRect.top = OriginalPlotRect.top + row* height;
			PhaseRect.bottom   = OriginalPlotRect.top + (row+1)* height;

			m_SelectedPhaseNumber = p;


			// update m_PlotRect for drawing links
			m_PlotRect = PhaseRect;


			BOOL bMovementIncluded = false;
			DTANode* pNode  = m_pDoc->m_NodeNoMap [m_CurrentNodeID];


			for (unsigned int i=0;i< pNode->m_MovementDataMap["ALLDAY"].m_MovementVector .size();i++)
			{

				DTANodeMovement movement = pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[i];

				bMovementIncluded = m_pDoc->IfMovementIncludedInPhase(m_CurrentNodeNumber ,  m_pDoc->m_TimingPlanVector[ m_SelectedTimingPlanNo].timing_plan_name,m_SelectedPhaseNumber, movement.in_link_from_node_id,movement.out_link_to_node_id );

				if(bMovementIncluded)
					break;
			}
			// draw phase diagram when there is movement
			if(bMovementIncluded)

			{
				DrawMovements(&dc,PhaseRect, true);
				int row = 5;
				float GreenStartTime = atof(m_PhasingGrid.GetItemText(row,p));

				row = 6;
				float PhaseEndTime = atof(m_PhasingGrid.GetItemText(row,p));

				if(PhaseEndTime <= GreenStartTime +1)  //skip empty movement
					continue;

				row = 3;
				float Yellow = atof(m_PhasingGrid.GetItemText(row,p));

				row =4;
				float AllRed = atof(m_PhasingGrid.GetItemText(row,p));


				if(m_RingTypeComboBox.GetCurSel () ==0)  //single ring
					DrawPhaseGreenTimeBand(&dc,GreenTimePlotRect,m_CycleLength,GreenStartTime,PhaseEndTime, Yellow, AllRed);
				else
				{
					if(p<=4)
						DrawPhaseGreenTimeBand(&dc,GreenTimePlotRect,m_CycleLength,GreenStartTime,PhaseEndTime, Yellow, AllRed);
					else
						DrawPhaseGreenTimeBand(&dc,GreenTimePlotRect2,m_CycleLength,GreenStartTime,PhaseEndTime, Yellow, AllRed);

				}
			}

		}

		m_SelectedPhaseNumber= 0;

	}

}

void CPage_Node_Phase::DrawPhaseGreenTimeBand(CPaintDC* pDC,CRect PlotRect, int CycleLength, float PhaseStartTime,float PhaseEndTime, float Yellow, float AllRed )
{

	CBrush  GreenBrush(RGB(0,255,0)); 
	CBrush  YellowBrush(RGB(255,255,0)); 

	pDC->SetBkMode(TRANSPARENT);
	pDC->SelectObject(&GreenBrush);

	float width = 	PlotRect.Width ();

	float GreenEndTime = PhaseEndTime - AllRed - Yellow;
	CRect PlotRectNew = PlotRect;

	PlotRectNew.left = PlotRect.left + PhaseStartTime/CycleLength * width;
	PlotRectNew.right  =  PlotRect.left + GreenEndTime/CycleLength *width;


	pDC->Rectangle (PlotRectNew);

	CString str;

	str.Format("P%d [%.1f s]",m_SelectedPhaseNumber,PhaseEndTime - PhaseStartTime );

	pDC->TextOutA(PlotRectNew.left+5,PlotRectNew.top +5,str);

	// yellow block
	pDC->SelectObject(&YellowBrush);

	PlotRectNew.left = PlotRectNew.right; // next to previous green block 
	PlotRectNew.right  =  PlotRectNew.left + Yellow/CycleLength *width;


	pDC->Rectangle (PlotRectNew);

}


void CPage_Node_Phase::DrawMovements(CPaintDC* pDC,CRect PlotRect, bool bPhaseWindow =false)
{

	m_MovementBezierVector.clear();

	CPen NormalPen(PS_SOLID,2,RGB(0,0,0));
	CPen TimePen(PS_DOT,1,RGB(0,0,0));

	CPen DASHPen(PS_SOLID,1,RGB(255,178,102));

	CPen DataPen(PS_SOLID,0,RGB(0,0,0));
	CPen SelectedPen(PS_SOLID,3,RGB(255,0,0));
	CPen SelectedPhasePen(PS_SOLID,4,RGB(0,0,255));

	CBrush  WhiteBrush(RGB(255,255,255)); 

	pDC->SetBkMode(TRANSPARENT);
	pDC->SelectObject(&DataPen);
	pDC->SelectObject(&WhiteBrush);

	pDC->Rectangle (PlotRect);

	CString str;

	if(bPhaseWindow == true )
	{
		str.Format("Phase %d",m_SelectedPhaseNumber);
		pDC->TextOutA(PlotRect.left+10,PlotRect.top +5,str);


	}


	CBrush  BrushLinkBand(RGB(152,245,255)); 
	pDC->SelectObject(&BrushLinkBand);

	DTANode* pNode  = m_pDoc->m_NodeNoMap [m_CurrentNodeID];

	int node_size = 10;
	int node_set_back = 50;

	int link_length = 100;
	int lane_width = 10;
	int text_length = link_length+ 20;

	str.Format("%d",m_CurrentNodeNumber);

	if(bPhaseWindow == false || m_SelectedPhaseNumber == 1)
		pDC->TextOutA( PlotRect.CenterPoint().x-5, PlotRect.CenterPoint().y-5,str);

	for (unsigned int i=0;i< pNode->m_MovementDataMap["ALLDAY"].m_MovementVector .size();i++)
	{
		DTANodeMovement movement = pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[i];

		if( m_pDoc->m_hide_non_specified_movement_on_freeway_and_ramp && movement.bNonspecifiedTurnDirectionOnFreewayAndRamps && i != m_SelectedMovementIndex)
			continue;



		DTALink* pInLink  = m_pDoc->m_LinkNoMap [movement.IncomingLinkNo];
		DTALink* pOutLink  = m_pDoc->m_LinkNoMap [movement.OutgoingLinkNo ];

		GDPoint p1, p2, p3;
		// 1: fetch all data
		p1  = m_pDoc->m_NodeNoMap[movement.in_link_from_node_id ]->pt;
		p2  = m_pDoc->m_NodeNoMap[movement.in_link_to_node_id ]->pt;
		p3  = m_pDoc->m_NodeNoMap[movement.out_link_to_node_id]->pt;

		double DeltaX = p2.x - p1.x ;
		double DeltaY = p2.y - p1.y ;
		double theta = atan2(DeltaY, DeltaX);

		GDPoint p1_new, p2_new, p3_new;
		GDPoint p1_text, p3_text;
		GDPoint pt_movement[3];


		// 2. set new origin
		p2_new.x = (-1)*node_set_back*cos(theta);  
		p2_new.y = (-1)*node_set_back*sin(theta);


		int link_mid_offset  = (pInLink->m_NumberOfLanes/2 +1)*lane_width;  // mid

		pt_movement[0].x = p2_new.x + link_mid_offset* cos(theta-PI/2.0f);
		pt_movement[0].y = p2_new.y + link_mid_offset* sin(theta-PI/2.0f);

		// 3 determine the control point for  PolyBezier
		float control_point_ratio = 0;
		if(movement.movement_turn == DTA_Through ) 
		{
			control_point_ratio = 0;
		}else if(movement.movement_turn == DTA_LeftTurn ) 
		{
			control_point_ratio = 1.2;
		}else
		{
			control_point_ratio = 0.5;
		}

		pt_movement[1].x = pt_movement[0].x + node_set_back*control_point_ratio*cos(theta);
		pt_movement[1].y = pt_movement[0].y + node_set_back*control_point_ratio*sin(theta);


		if(bPhaseWindow == true)
		{
			link_length = 1;
			text_length = node_set_back+10;
		}

		p1_new.x = (-1)*link_length*cos(theta);
		p1_new.y = (-1)*link_length*sin(theta);


		p1_text.x= (-1)*(text_length)*cos(theta);
		p1_text.y= (-1)*(text_length)*sin(theta);

		// 4: draw from node name

		str.Format("%d",m_pDoc->m_NodeNoMap [movement.in_link_from_node_id]->m_NodeNumber );


		if(p1_text.y < -50)
			p1_text.y +=10;

		CPoint pt_text = NPtoSP(p1_text);

		if(bPhaseWindow == false  || m_SelectedPhaseNumber == 1)
		{
			pDC->SetTextColor(RGB(0,0,255));
			pDC->TextOutA(pt_text.x-10,pt_text.y,str);
			pDC->SetTextColor(RGB(0,0,0));
		}

		if(bPhaseWindow == false)
			DrawLink(pDC,p1_new,p2_new,pInLink->m_NumberOfLanes,theta,lane_width);

		////////////////////////////////////////////
		//5: outgoing link
		DeltaX = p3.x - p2.x ;
		DeltaY = p3.y - p2.y ;
		theta = atan2(DeltaY, DeltaX);

		// set new origin
		p2_new.x = node_set_back*cos(theta);  
		p2_new.y = node_set_back*sin(theta);

		link_mid_offset  = (pOutLink->m_NumberOfLanes/2+1)*lane_width;
		pt_movement[2].x = p2_new.x + link_mid_offset* cos(theta-PI/2.0f);
		pt_movement[2].y = p2_new.y + link_mid_offset* sin(theta-PI/2.0f);


		p3_new.x = link_length*cos(theta);
		p3_new.y = link_length*sin(theta);

		p3_text.x= text_length*cos(theta);
		p3_text.y= text_length*sin(theta);

		if(bPhaseWindow == false)
			DrawLink(pDC,p2_new,p3_new,pOutLink->m_NumberOfLanes,theta,lane_width);

		DTALink * pRevLink = NULL; //reversed link
		unsigned long ReversedLinkKey = m_pDoc->GetLinkKey(pOutLink->m_ToNodeID, pOutLink->m_FromNodeID);

		int reversed_link_id = 0;
		if ( m_pDoc->m_NodeNotoLinkMap.find ( ReversedLinkKey) == m_pDoc->m_NodeNotoLinkMap.end())
		{
			str.Format("%d",m_pDoc->m_NodeNoMap [movement.out_link_to_node_id ]->m_NodeNumber );

			if(p3_text.y < -50)
				p3_text.y +=10;

			pt_text = NPtoSP(p3_text);

			if(bPhaseWindow == false  || m_SelectedPhaseNumber == 1)
				pDC->TextOutA(pt_text.x-10,pt_text.y,str);

		}



		// draw movement 

		CPoint Point_Movement[4];



		if(i == m_SelectedMovementIndex)
		{
			pDC->SelectObject(&SelectedPen);
		}
		else
		{
			//if(m_ListCtrl.GetItemText (i,4).Find("Prohibited") != -1) 			// select prohibited movement pen: 
			//	pDC->SelectObject(&g_PenProhibitedMovement);
			//else
			//	pDC->SelectObject(&NormalPen);
		}

		if(movement.movement_turn == DTA_LeftTurn )
		{
			float weight = 0.9;
			pt_movement[1].x = (1-weight)*(pt_movement[0].x + pt_movement[2].x)/2 +0*weight;
			pt_movement[1].y = (1-weight)*(pt_movement[0].y + pt_movement[2].y)/2 +0*weight;
		}

		Point_Movement[0]= NPtoSP(pt_movement[0]);
		Point_Movement[1]= NPtoSP(pt_movement[1]);
		Point_Movement[2]= NPtoSP(pt_movement[1]);
		Point_Movement[3]= NPtoSP(pt_movement[2]);

		Point_Movement[0]= NPtoSP(pt_movement[0]);
		Point_Movement[1]= NPtoSP(pt_movement[1]);
		Point_Movement[2]= NPtoSP(pt_movement[1]);
		Point_Movement[3]= NPtoSP(pt_movement[2]);

		MovementBezier element(Point_Movement[0], Point_Movement[1],Point_Movement[3]);

		m_MovementBezierVector.push_back (element);

		if(m_bHideRightTurnMovement &&( pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[i].movement_turn == DTA_RightTurn ||  pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[i].movement_turn == DTA_RightTurn2))
			continue;

		bool bMovementIncluded = false;

		if(m_SelectedPhaseNumber >=1)
		{
			bMovementIncluded = m_pDoc->IfMovementIncludedInPhase(m_CurrentNodeNumber ,m_pDoc->m_TimingPlanVector [ m_SelectedTimingPlanNo].timing_plan_name,m_SelectedPhaseNumber, movement.in_link_from_node_id,movement.out_link_to_node_id );
		}

		if(m_SelectedPhaseNumber <=0 ||  /* all phases*/
			m_SelectedPhaseNumber>=1 && ( bMovementIncluded == true || m_bPhasingDataEditMode) )  // seleted phase
		{


			pDC->SelectObject(&NormalPen);

			if(m_SelectedPhaseNumber>=1 && m_bPhasingDataEditMode)
			{
				if(  bMovementIncluded == true)
					pDC->SelectObject(&NormalPen);
				else
					pDC->SelectObject(&DASHPen);

			}

			if(i == m_SelectedMovementIndex && m_bPhasingDataEditMode)
			{
				pDC->SelectObject(&SelectedPen);
			}

			//overwrite
			if(i == m_SelectedMovementIndex)
			{
				pDC->SelectObject(&SelectedPen);
			}

			pDC->PolyBezier(Point_Movement,4);

			bool bShowArrow = false;

			if(m_SelectedPhaseNumber>=1 &&  bMovementIncluded == true)
			{
				bShowArrow = true;

			}
			if(bShowArrow)
			{
				CPoint FromPoint = Point_Movement[2] ; 
				CPoint ToPoint = Point_Movement[3];


				CPoint arrow_pts[3];
				double slopy = atan2((double)(FromPoint.y - ToPoint.y), (double)(FromPoint.x - ToPoint.x));
				double cosy = cos(slopy);
				double siny = sin(slopy);   
				double display_length  = sqrt((double)(FromPoint.y - ToPoint.y)*(FromPoint.y - ToPoint.y)+(double)(FromPoint.x - ToPoint.x)*(FromPoint.x - ToPoint.x));
				double arrow_size = min(10,display_length/3.0);

				if(arrow_size>0.2)
				{

					arrow_pts[0] = ToPoint;
					arrow_pts[1].x = ToPoint.x + (int)(arrow_size * cosy - (arrow_size / 2.0 * siny) + 0.5);
					arrow_pts[1].y = ToPoint.y + (int)(arrow_size * siny + (arrow_size / 2.0 * cosy) + 0.5);
					arrow_pts[2].x = ToPoint.x + (int)(arrow_size * cosy + arrow_size / 2.0 * siny + 0.5);
					arrow_pts[2].y = ToPoint.y - (int)(arrow_size / 2.0 * cosy - arrow_size * siny + 0.5);

					pDC->Polygon(arrow_pts, 3);

				}
			}



		}
		//restore pen
		pDC->SelectObject(&DataPen);

	}
}

void CPage_Node_Phase::DrawLink(CPaintDC* pDC,GDPoint pt_from, GDPoint pt_to,int NumberOfLanes, double theta, int lane_width)
{
	CPoint DrawPoint[4];

	//then offset
	int link_offset = lane_width;

	pt_from.x += link_offset* cos(theta-PI/2.0f);
	pt_to.x += link_offset* cos(theta-PI/2.0f);

	pt_from.y += link_offset* sin(theta-PI/2.0f);
	pt_to.y += link_offset* sin(theta-PI/2.0f);

	DrawPoint[0] = NPtoSP(pt_from);
	DrawPoint[1] = NPtoSP(pt_to);

	link_offset = min(5,NumberOfLanes)*lane_width ;
	pt_from.x += link_offset* cos(theta-PI/2.0f);
	pt_to.x += link_offset* cos(theta-PI/2.0f);

	pt_from.y += link_offset* sin(theta-PI/2.0f);
	pt_to.y += link_offset* sin(theta-PI/2.0f);

	DrawPoint[2] = NPtoSP(pt_to);
	DrawPoint[3] = NPtoSP(pt_from);

	pDC->Polygon(DrawPoint, 4);

}
// CPage_Node_Phase message handlers

void CPage_Node_Phase::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	unsigned int i;

	DTANode* pNode  = m_pDoc->m_NodeNoMap [m_CurrentNodeID];


	m_SelectedMovementIndex =  FindClickedMovement(point);

	CPropertyPage::OnLButtonDown(nFlags, point);
	Invalidate();

}


void CPage_Node_Phase::SaveData()
{

	bool bTurnVolumeModified = false;
	UpdateData(1);

	std::map<int, int> IncomingLinkMap;

	DTANode* pNode  = m_pDoc->m_NodeNoMap [m_CurrentNodeID];

	if(	pNode->m_CycleLengthInSecond  != m_CycleLength || pNode->m_SignalOffsetInSecond   != m_Offset)
	{
		m_pDoc->Modify (true);

		pNode->m_SignalOffsetInSecond   = m_Offset;
	}

	pNode->m_ControlType  = m_ControlTypeVector[m_ControlTypeComboBox.GetCurSel ()];





	std::string timing_plan_name = m_pDoc-> m_TimingPlanVector [m_SelectedTimingPlanNo].timing_plan_name;

	DTA_Phasing_Data_Matrix element = m_pDoc->GetPhaseData(m_CurrentNodeNumber ,timing_plan_name);

	// save phasing data
	if(m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_ControlType == m_pDoc->m_ControlType_PretimedSignal
		|| m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_ControlType == m_pDoc->m_ControlType_ActuatedSignal)
	{

		int row = 1;

		int time = 0;

		int start_time = 0;
		int end_time = 0;

		for(int p=1; p<=8; p++)
		{
			start_time = time;

			row = 1;  // minGreen
			m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_MinGreen,m_PhasingGrid.GetItemText(row,p));

			row = 2;  // maxGreen
			m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_MaxGreen,m_PhasingGrid.GetItemText(row,p));

			int max_green  = atoi( m_PhasingGrid.GetItemText(row,p));
			row = 3;  // yellow
			m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_Yellow,m_PhasingGrid.GetItemText(row,p));

			row = 4;  // all red
			m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_AllRed,m_PhasingGrid.GetItemText(row,p));

			row = 2;
			time += atof( m_PhasingGrid.GetItemText(row++,p));
			end_time = time;


			m_pDoc->SaveNodeFile();


			if(element.GetString (DTA_SIG_PHASE_VALUE, TIMING_RingType).Find("Single")!=-1 && max_green >=1)  // single ring
			{

				m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_Start,start_time);
				m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_End,end_time);

				//for each movement included:


			}else
			{
				row = 5;
				start_time =  atof( m_PhasingGrid.GetItemText(row++,p));
				end_time =  atof( m_PhasingGrid.GetItemText(row,p));

				m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_Start,start_time);
				m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_End,end_time);



				// do nothing for (complex) double ring
			}


		}	



		m_pDoc->SetupSignalValue(m_CurrentNodeNumber , timing_plan_name,TIMING_CycleLength,m_CycleLength);

		CString str_ring_type;
		m_RingTypeComboBox.GetLBText(m_RingTypeComboBox.GetCurSel(),str_ring_type);
		m_pDoc->SetupSignalValue(m_CurrentNodeNumber , timing_plan_name,TIMING_RingType,str_ring_type);

		m_pDoc->SetupSignalValue(m_CurrentNodeNumber , timing_plan_name,TIMING_Offset,m_Offset);



	}

		CString optimization_method_type;
		
		if(m_bOptimizationMethod)
			optimization_method_type = "yes";
		else
			optimization_method_type = "no";

	// for all timing plans
	for(int tp  = 0; tp <  m_pDoc-> m_TimingPlanVector.size(); tp++)
	{
	std::string timing_plan_name = m_pDoc-> m_TimingPlanVector [tp].timing_plan_name;

	m_pDoc->SetupSignalValue(m_CurrentNodeNumber , timing_plan_name,TIMING_OptimizationMethod,optimization_method_type);
	}

	m_pDoc->UpdateMovementGreenStartAndEndTimeFromPhasingData(m_CurrentNodeNumber, timing_plan_name);
}

void CPage_Node_Phase::OnOK( )
{
	SaveData();
	CPropertyPage::OnOK();
}

void CPage_Node_Phase::OnCancel( )
{
	int row = 1;
	for(int tp = 0; tp< m_pDoc-> m_TimingPlanVector.size(); tp++)
	{

		std::string timing_plan_name = m_pDoc-> m_TimingPlanVector[tp].timing_plan_name;  // fetch timing_plan (unique) name

		if(m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_ControlType == m_pDoc->m_ControlType_PretimedSignal
			|| m_pDoc->m_NodeNoMap [m_CurrentNodeID]->m_ControlType == m_pDoc->m_ControlType_ActuatedSignal)

		{
			for(int p=1; p<=8; p++)
			{
				//reset using original string of movement vector
				m_pDoc->SetupPhaseData(m_CurrentNodeNumber , timing_plan_name, p, PHASE_MOVEMENT_VECTOR,MovementVectorString[p-1]);
			}

		}
	}
	CPropertyPage::OnCancel();
}

void CPage_Node_Phase::OnBnClickedButtonSave()
{


}

void CPage_Node_Phase::RunQEM()
{	
	UpdateData(1);
	SaveData();
	m_pDoc->RunQEMTool(m_CurrentNodeNumber, m_bUsingObsHourlyCount);

	DTA_Phasing_Data_Matrix element = m_pDoc->GetPhaseData(m_CurrentNodeNumber ,m_pDoc->m_TimingPlanVector[m_SelectedTimingPlanNo].timing_plan_name);
	m_CycleLength = atoi(element.GetString (DTA_SIG_PHASE_VALUE, TIMING_CycleLength));

	UpdateList();

	m_pDoc->Modify (true);

	UpdatePhaseData();

	UpdateData(0);
	Invalidate();

}

void CPage_Node_Phase::OnBnClickedButtonQem()
{	
	UpdateData(1);
	if(m_bUsingObsHourlyCount)
	{
	OnBnClickedButtonQem2();
	return;
	}

	DTANode* pNode  = m_pDoc->m_NodeNoMap [m_CurrentNodeID];

	bool SimuTurnVolume = false;

for(int tp = 0; tp<  m_pDoc->m_TimingPlanVector.size(); tp++)  // first loop for each timing plan
{
	for (unsigned int i=0;i< pNode->m_MovementDataMap[ m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector .size();i++)
	{
		if(pNode->m_MovementDataMap[ m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector[i].sim_turn_count  >0)
		{

			SimuTurnVolume = true;
			break;

		}
	}
}
	if(SimuTurnVolume == false && m_bUsingObsHourlyCount == false)
	{
		AfxMessageBox("Simulated turning movement Counts are not available. ",MB_ICONINFORMATION);
		return;
	}
	for(int tp = 0; tp<  m_pDoc->m_TimingPlanVector.size(); tp++)  // first loop for each timing plan
	{
		for (unsigned int i=0;i< pNode->m_MovementDataMap[ m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector .size();i++)
		{
		
			
			float number_of_hours = max(0.01,(m_pDoc->m_TimingPlanVector[tp].end_time_in_min - m_pDoc->m_TimingPlanVector[tp]. start_time_in_min )/60.0);
			float sim_turn_hourly_count = pNode->m_MovementDataMap[ m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector[i].sim_turn_count/ number_of_hours;
			pNode->m_MovementDataMap[ m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector[i].QEM_TurnVolume  = sim_turn_hourly_count  ;
		}
	}

	RunQEM();
}


void CPage_Node_Phase::OnBnClickedButtonQem2()
{
	DTANode* pNode  = m_pDoc->m_NodeNoMap [m_CurrentNodeID];

	bool ObservedTurnVolume = false;
	for (unsigned int i=0;i< pNode->m_MovementDataMap["ALLDAY"].m_MovementVector .size();i++)
	{
		if(pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[i].obs_turn_hourly_count >0)
		{

			ObservedTurnVolume = true;
			break;

		}
	}

	if(ObservedTurnVolume == false && m_bUsingObsHourlyCount)
	{
		AfxMessageBox("Observed turning movement counts are not available. ",MB_ICONINFORMATION);
		return;
	}
for(int tp = 0; tp<  m_pDoc->m_TimingPlanVector.size(); tp++)  // first loop for each timing plan
{
	for (unsigned int i=0;i< pNode->m_MovementDataMap[ m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector .size();i++)
	{
		pNode->m_MovementDataMap[m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector[i].QEM_TurnVolume  =
			pNode->m_MovementDataMap[m_pDoc->m_TimingPlanVector[tp].timing_plan_name].m_MovementVector[i].obs_turn_hourly_count;
	}
}
	RunQEM();
}


void CPage_Node_Phase::OnLbnSelchangeList1()
{
	Invalidate();
}
void CPage_Node_Phase::OnLbnDblclkList1()
{
	Invalidate();
}

void CPage_Node_Phase::DisplayPhasingGrid()
{

	m_PhasingGrid.SetRowCount(9);  // Phase Number, Green Time 
	m_PhasingGrid.SetColumnCount(9);  // Title, green time value

	m_PhasingGrid.SetFixedColumnCount(1);
	//	m_PhasingGrid.SetFixedRowCount(1);

	m_PhasingGrid.SetFixedColumnSelection(true);


	CString str;

	int p;
	int i = 0;  // Phase Number
	for(p = 1; p <= 8; p++)
	{
		str.Format("Phase %d", p);
		SetItemTextInPhasingGrid(i,p,str,true);
		m_PhasingGrid.SetItemBkColour(i,p,RGB(240,240,240));
	}


	i=1;  // Green Time
	p = 0;

	std::string timing_plan_name = m_pDoc->m_TimingPlanVector[m_SelectedTimingPlanNo].timing_plan_name;

	DTA_Phasing_Data_Matrix element = m_pDoc->GetPhaseData(m_CurrentNodeNumber ,timing_plan_name);


	SetItemTextInPhasingGrid(i,0,"MinGreen");

	for(p = 1; p <= 8; p++)
	{
		SetItemTextInPhasingGrid(i,p,element.GetString  ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_MinGreen));
	}

	i++;

	SetItemTextInPhasingGrid(i,0,"MaxGreen");

	for(p = 1; p <= 8; p++)
	{
		SetItemTextInPhasingGrid(i,p,element.GetString ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_MaxGreen));
	}

	i++;

	SetItemTextInPhasingGrid(i,0,"Yellow");

	for(p = 1; p <= 8; p++)
	{
		SetItemTextInPhasingGrid(i,p,element.GetString ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_Yellow));
	}

	i++;

	SetItemTextInPhasingGrid(i,0,"AllRed");


	for(p = 1; p <= 8; p++)
	{
		SetItemTextInPhasingGrid(i,p,element.GetString ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_AllRed));
	}
	i++;


	SetItemTextInPhasingGrid(i,0,"Start");

	for(p = 1; p <= 8; p++)
	{
		SetItemTextInPhasingGrid(i,p,element.GetString ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_Start));
	}
	i++;

	SetItemTextInPhasingGrid(i,0,"End");

	for(p = 1; p <= 8; p++)
	{
		SetItemTextInPhasingGrid(i,p,element.GetString ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_End));
	}


	i++;

	SetItemTextInPhasingGrid(i,0,"MovementVector");


	for(p = 1; p <= 8; p++)
	{
		SetItemTextInPhasingGrid(i,p,element.GetString ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_MOVEMENT_VECTOR));
	}

	// get movement vector as back up value
	for(p = 1; p <= 8; p++)
	{
		MovementVectorString.push_back(element.GetString ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+p), PHASE_MOVEMENT_VECTOR));
	}

}


void CPage_Node_Phase::UpdatePhasingDataInGrid()
{

	DTA_Phasing_Data_Matrix element = m_pDoc->GetPhaseData(m_CurrentNodeNumber ,m_pDoc->m_TimingPlanVector[m_SelectedTimingPlanNo].timing_plan_name);

	m_CycleLength = 0;
	int time = 0;

	int start_time = 0;
	int end_time = 0;

	int row = 0;
	for(int p=1; p<=8; p++)
	{
		start_time = time;

		row = 2;
		time += atof( m_PhasingGrid.GetItemText(row,p));

		int max_green = atoi( m_PhasingGrid.GetItemText(row++,p));
		time += atof( m_PhasingGrid.GetItemText(row++,p));
		time += atof( m_PhasingGrid.GetItemText(row++,p));

		end_time = time;


		if(element.GetString (DTA_SIG_PHASE_VALUE, TIMING_RingType).Find("Single")!=-1 && max_green >=1)  // single ring
		{
			CString str;
			str.Format("%d", start_time);

			SetItemTextInPhasingGrid(5,p, str);
			str.Format("%d", end_time);
			SetItemTextInPhasingGrid(6,p, str);

			m_CycleLength = time;

		}else
		{
			//dual ring 

			int row = 6;
			int end_time = atoi( m_PhasingGrid.GetItemText(row,p));

			m_CycleLength = max(end_time, m_CycleLength);
		}

	}	


	UpdateData(0);


}

void CPage_Node_Phase::OnGridEndSelChange(NMHDR *pNotifyStruct, LRESULT* /*pResult*/)
{
	NM_GRIDVIEW* pItem = (NM_GRIDVIEW*) pNotifyStruct;
	TRACE(_T("End Selection Change on row %d, col %d \n"), 
		pItem->iRow, pItem->iColumn );

	//	if(pItem->iRow ==-1) // column selecetd
	{
		m_SelectedPhaseNumber = pItem->iColumn;
	//	UpdatePhasingDataInGrid();
		Invalidate();
	}


}


void CPage_Node_Phase::OnBnClickedEditMode()
{
	UpdateData(true);
	Invalidate();
}

void CPage_Node_Phase::OnContextMenu(CWnd* pWnd, CPoint point)
{

}
void CPage_Node_Phase::OnLButtonDblClk(UINT nFlags, CPoint point)
{

	if(m_bPhasingDataEditMode)
	{
		m_SelectedMovementIndex =  FindClickedMovement(point);

		if(m_SelectedPhaseNumber>=1 && m_SelectedMovementIndex>=0)
		{
			DTANode* pNode  = m_pDoc->m_NodeNoMap [m_CurrentNodeID];

			//Get Current status
			BOOL bMovementIncluded = m_pDoc->IfMovementIncludedInPhase(m_CurrentNodeNumber ,
				m_pDoc->m_TimingPlanVector[m_SelectedTimingPlanNo].timing_plan_name,m_SelectedPhaseNumber, 
				pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].in_link_from_node_id,
				pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].out_link_to_node_id );

			DTA_Phasing_Data_Matrix element = m_pDoc->GetPhaseData(m_CurrentNodeNumber ,m_pDoc->m_TimingPlanVector[m_SelectedTimingPlanNo].timing_plan_name);

			CString movement_vector = element.GetString  ( (DTA_SIG_PHASE)(DTA_SIG_PHASE_VALUE+ m_SelectedPhaseNumber), PHASE_MOVEMENT_VECTOR);
			CString sub_movement_str;

			sub_movement_str.Format("%d:%d;", m_pDoc->m_NodeNoMap[pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].in_link_from_node_id]->m_NodeNumber,
				m_pDoc->m_NodeNoMap[pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].out_link_to_node_id]->m_NodeNumber);

			if(bMovementIncluded)
			{
				movement_vector.Replace(sub_movement_str,_T(""));

				m_MovementMsg.Format ("Movement %d->%d->%d has been deleted from phase %d.", m_pDoc->m_NodeNoMap[pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].in_link_from_node_id]->m_NodeNumber,
					m_CurrentNodeNumber,m_pDoc->m_NodeNoMap[pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].out_link_to_node_id]->m_NodeNumber,
					m_SelectedPhaseNumber);

			}
			else
			{
				//select
				movement_vector+= sub_movement_str;
				m_MovementMsg.Format ("Movement %d->%d->%d has been added into phase %d.", m_pDoc->m_NodeNoMap[pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].in_link_from_node_id]->m_NodeNumber,
					m_CurrentNodeNumber,m_pDoc->m_NodeNoMap[pNode->m_MovementDataMap["ALLDAY"].m_MovementVector[m_SelectedMovementIndex].out_link_to_node_id]->m_NodeNumber,
					m_SelectedPhaseNumber);



			}

			//update movement vector string
			m_pDoc->SetupPhaseData(m_CurrentNodeNumber,m_pDoc->m_TimingPlanVector[m_SelectedTimingPlanNo].timing_plan_name,m_SelectedPhaseNumber, PHASE_MOVEMENT_VECTOR, movement_vector);
			UpdateData(false);
			Invalidate();

		}
	}

	CPropertyPage::OnLButtonDblClk(nFlags, point);
}

void CPage_Node_Phase::OnCbnSelchangeComboTimingPlan()
{

}


void CPage_Node_Phase::OnCbnSelchangeCombo1()
{
	// TODO: Add your control notification handler code here
}

void CPage_Node_Phase::OnCbnSelchangeCombo2()
{
	// TODO: Add your control notification handler code here
}

void CPage_Node_Phase::OnBnClickedEditMode2()
{
	UpdateData(1);
	Invalidate();
}

void CPage_Node_Phase::OnCbnSelchangeComboPhasemovementdiagram()
{
	Invalidate();
}

void CPage_Node_Phase::OnBnClickedButtonQemView()
{
	// TODO: Add your control notification handler code here
}

void CPage_Node_Phase::OnBnClickedCheckMultiplePhaseDiagram()
{

	UpdateData(1);
	Invalidate();


}

void CPage_Node_Phase::OnBnClickedCheckOptimizationmethod()
{
	UpdateData(1);
	GetDlgItem(IDC_BUTTON_QEM) ->EnableWindow(m_bOptimizationMethod);

}

void CPage_Node_Phase::OnBnClickedCheckOptimizationObs()
{
	// TODO: Add your control notification handler code here
}
