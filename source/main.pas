unit Main;

{$mode objfpc}{$H+}
{$WARN 4080 off : Converting the operands to "$1" before doing the subtract could prevent overflow errors.}
interface

uses
  Classes, SysUtils, Forms, Controls, Math, IniFiles,
  Graphics, GraphUtil, FPImage, IntfGraphics,
  LCLType, LCLIntf, Dialogs, ExtCtrls, StdCtrls, Buttons, ExtDlgs, Spin,
  Delaunay;

type

  { TMainForm }

  TMainForm = class(TForm)
    btnBrowse: TBitBtn;
    btnLoad: TBitBtn;
    btnApplyLloydsAlgorithm: TButton;
    btnNewSeedPoints: TButton;
    cbFileName: TComboBox;
    cbDrawSeedPoints: TCheckBox;
    cbDrawDelaunayBorder: TCheckBox;
    cbShowOrigImage: TCheckBox;
    cbFillWithColorOfSeedPoints: TCheckBox;
    cbDrawVoronoiBorder: TCheckBox;
    cbDelaunayGradient: TCheckBox;
    cbColorFromImage: TCheckBox;
    clbVoronoiBorderColor: TColorButton;
    clbDelaunayBorderColor: TColorButton;
    gbOrigImage: TGroupBox;
    gbSeedPoints: TGroupBox;
    gbDelaunayTriangles: TGroupBox;
    gbVoronoiPolygons: TGroupBox;
    gbLloydsAlgorithm: TGroupBox;
    Label1: TLabel;
    ImageBox: TPaintBox;
    lblNumSeedPoints: TLabel;
    OpenPictureDialog: TOpenPictureDialog;
    Panel1: TPanel;
    Panel2: TPanel;
    rbConstantSeedPointSize: TRadioButton;
    rbVaryingSeedPointSize: TRadioButton;
    seNumSeedPoints: TSpinEdit;
    seMaxSeedPtSize: TSpinEdit;
    procedure btnApplyLloydsAlgorithmClick(Sender: TObject);
    procedure btnBrowseClick(Sender: TObject);
    procedure btnLoadClick(Sender: TObject);
    procedure btnNewSeedPointsClick(Sender: TObject);
    procedure cbFileNameCloseUp(Sender: TObject);
    procedure cbUseLloydsAlgorithmChange(Sender: TObject);
    procedure FormCloseQuery(Sender: TObject; var CanClose: Boolean);
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure ImageBoxPaint(Sender: TObject);
    procedure seLloydsIterationsChange(Sender: TObject);
    procedure seNumSeedPointsChange(Sender: TObject);
    procedure UpdateImageHandler(Sender: TObject);
  private
    FImg: TLazIntfImage;
    FPicture: TPicture;
    FDrawBitmap: TBitmap;
    FLloydsAlgorithmNeeded: Boolean;
    FSeedPoints: array of TDoublePoint;
    FTriangulation: TTriangulation;
    procedure InitSeedPoints;
    procedure LoadFile(AFileName: String);
    procedure LloydsAlgorithm;
    procedure UpdateFileHistory(AFileName: String);
    procedure UpdateImage;
    procedure ReadIni;
    procedure WriteIni;

  public

  end;

var
  MainForm: TMainForm;

implementation

{$R *.lfm}

const
  MAX_HISTORY_COUNT = 20;

function CreateIni: TCustomIniFile;
var
  fn: String;
begin
  fn := ChangeFileExt(Application.ExeName, '.ini');
  Result := TIniFile.Create(fn);
end;

{ Set font style of Groupbox caption to bold, but keep items normal }
procedure BoldGroup(AControl: TCustomGroupBox);
var
  i: Integer;
begin
  AControl.Font.Style := [fsBold];
  for i:=0 to AControl.ControlCount-1 do
    AControl.Controls[i].Font.Style := [];
end;


{ TMainForm }

procedure TMainForm.btnApplyLloydsAlgorithmClick(Sender: TObject);
begin
  Screen.Cursor := crHourGlass;
  try
    FLloydsAlgorithmNeeded := true;
    InitSeedPoints;
    UpdateImage;
  finally
    Screen.Cursor := crDefault;
  end;
end;

procedure TMainForm.btnBrowseClick(Sender: TObject);
begin
  OpenPictureDialog.InitialDir := ExtractFileDir(cbFileName.Text);
  OpenPictureDialog.FileName := ExtractFileName(cbFileName.Text);
  if OpenPictureDialog.Execute then
  begin
    cbFileName.Text := OpenPictureDialog.Filename;
    btnLoadClick(nil);
  end;
end;

procedure TMainForm.btnLoadClick(Sender: TObject);
begin
  if FileExists(cbFileName.Text) then
    LoadFile(cbFileName.Text);
end;

procedure TMainForm.btnNewSeedPointsClick(Sender: TObject);
begin
  Screen.Cursor := crHourglass;
  try
    FSeedPoints := nil;
    InitSeedPoints;
    UpdateImage;
  finally
    Screen.Cursor := crDefault;
  end;
end;

procedure TMainForm.cbFileNameCloseUp(Sender: TObject);
begin
  if cbFileName.ItemIndex > -1 then
  begin
    cbFileName.Text := cbFileName.Items[cbFileName.ItemIndex];
    LoadFile(cbFileName.Text);
  end;
end;

procedure TMainForm.UpdateImageHandler(Sender: TObject);
begin
  Screen.Cursor := crHourglass;
  try
    UpdateImage;
  finally
    Screen.Cursor := crDefault;
  end;
end;

procedure TMainForm.cbUseLloydsAlgorithmChange(Sender: TObject);
begin
  InitSeedPoints;
  UpdateImage;
end;

procedure TMainForm.FormCloseQuery(Sender: TObject; var CanClose: Boolean);
begin
  if CanClose then
    try
      WriteIni;
    except
    end;
end;

procedure TMainForm.FormCreate(Sender: TObject);
begin
  BoldGroup(gbOrigImage);
  BoldGroup(gbSeedPoints);
  BoldGroup(gbDelaunayTriangles);
  BoldGroup(gbVoronoiPolygons);
  BoldGroup(gbLloydsAlgorithm);

  ReadIni;
  if ParamCount > 0 then
  begin
    cbFileName.Text := ParamStr(1);
    LoadFile(ParamStr(1));
  end;
end;

procedure TMainForm.FormDestroy(Sender: TObject);
begin
  FTriangulation.Free;
  FDrawBitmap.Free;
  FPicture.Free;
  FImg.Free;
end;

procedure TMainForm.ImageBoxPaint(Sender: TObject);
var
  x, y: Integer;
begin
  if (FDrawBitmap = nil) or (FDrawBitmap.Width = 0) or (FDrawBitmap.Height = 0) then
    exit;
  x := (ImageBox.Width - FDrawBitmap.Width) div 2;
  y := (Imagebox.Height - FDrawBitmap.Height) div 2;
  ImageBox.Canvas.Draw(x, y, FDrawBitmap);
end;

procedure TMainForm.seLloydsIterationsChange(Sender: TObject);
begin
  UpdateImage;
end;

procedure TMainForm.InitSeedPoints;
var
  i: integer;
  n: Integer;
begin
  if FPicture = nil then
    exit;

  n := Length(FSeedPoints);
  SetLength(FSeedPoints, seNumSeedPoints.Value);
  if n < 4 then begin
    FSeedPoints[0] := DoublePoint(0, 0);
    FSeedPoints[1] := DoublePoint(FPicture.Width, 0);
    FSeedPoints[2] := DoublePoint(0, FPicture.Height);
    FSeedPoints[3] := DoublePoint(FPicture.Width, FPicture.Height);
    n := 4;
  end;
  for i := n to High(FSeedPoints) do
  begin
    FSeedPoints[i].X := Random * FPicture.Width;
    FSeedPoints[i].Y := Random * FPicture.Height;
  end;

  FTriangulation.Free;
  FTriangulation := TTriangulation.Create(@FSeedPoints[0], Length(FSeedPoints));
  FTriangulation.Execute;

  if FLloydsAlgorithmNeeded then
  begin
    LloydsAlgorithm;
    FTriangulation.Free;
    FTriangulation := TTriangulation.Create(@FSeedPoints[0], Length(FSeedPoints));
    FTriangulation.Execute;
    FLloydsAlgorithmNeeded := false;
  end;
end;

procedure TMainForm.UpdateImage;

  procedure DrawBackground(ACanvas: TCanvas; AWidth, AHeight: Integer);
  begin
    ACanvas.Brush.Style := bsSolid;
    ACanvas.Brush.Color := clWindow;
    ACanvas.FillRect(0, 0, AWidth, AHeight);
  end;

type
  TColoredPoint = record
    x, y: LongInt;
    Color: TFPColor;
  end;
  TTriangle = record
    i, j, k: Integer;
  end;
var
  i, j, j0, j1, j2: Integer;
  x, y: Integer;
  P: TPoint;
  Pts: array of TPoint = nil;
  cPts: array of TColoredPoint = nil;
  tri: array of TTriangle = nil;
  clr: TColor;
  c: TFPColor;
  value: Double;
  r: Integer;
begin
  if FDrawBitmap = nil then
  begin
    DrawBackground(ImageBox.Canvas, ImageBox.Width, ImageBox.Height);
    exit;
  end;

  Screen.Cursor := crHourGlass;
  try
    Application.ProcessMessages;

    if cbShowOrigImage.Checked then
      FDrawBitmap.Assign(FPicture.Bitmap)
    else
      DrawBackground(FDrawBitmap.Canvas, FDrawBitmap.Width, FDrawBitmap.Height);

    if FTriangulation <> nil then
    begin
      // *** Delaunay triangles ***
      // Fill triangles with a tri-color gradient
      if cbDelaunayGradient.Checked then
      begin
        SetLength(cPts, Length(FSeedPoints));
        for i := 0 to High(FSeedPoints) do
        begin
          x := EnsureRange(round(FSeedPoints[i].X), 0, FImg.Width-1);
          y := EnsureRange(round(FSeedPoints[i].Y), 0, FImg.Height-1);
          c := FImg.Colors[x, y];
          cPts[i].X := x;
          cPts[i].Y := y;
          cPts[i].Color := FImg.Colors[x, y];
        end;
        SetLength(tri, Length(FTriangulation.Triangles));
        for i := 0 to High(tri) do
        begin
          tri[i].i := FTriangulation.Triangles[i].VertexIndex[0];
          tri[i].j := FTriangulation.Triangles[i].VertexIndex[1];
          tri[i].k := FTriangulation.Triangles[i].VertexIndex[2];
        end;
        GradientFill(FDrawBitmap.Canvas.Handle, @cPts[0], Length(cPts), @tri[0], Length(tri), GRADIENT_FILL_TRIANGLE);
        cPts := nil;
      end;

      // Draw border of Delaunay triangles
      if cbDrawDelaunayBorder.Checked then
      begin
        SetLength(Pts, 3);
        FDrawBitmap.Canvas.Brush.Style := bsClear;
        FDrawBitmap.Canvas.Pen.Style := psSolid;
        FDrawBitmap.Canvas.Pen.Color := clbDelaunayBorderColor.ButtonColor;
        for i := 0 to High(FTriangulation.Triangles) do
        begin
          j0 := FTriangulation.Triangles[i].VertexIndex[0];
          j1 := FTriangulation.Triangles[i].VertexIndex[1];
          j2 := FTriangulation.Triangles[i].VertexIndex[2];
          Pts[0] := FTriangulation.Vertex[j0].Round;
          Pts[1] := FTriangulation.Vertex[j1].Round;
          Pts[2] := FTriangulation.Vertex[j2].Round;
          FDrawBitmap.Canvas.Polygon(Pts);
        end;
        Pts := nil;
      end;

      // *** Voronoi polygons ***
      if cbFillWithColorOfSeedPoints.Checked or cbDrawVoronoiBorder.Checked then
      begin
        if cbFillWithColorOfSeedPoints.Checked then
          FDrawBitmap.Canvas.Brush.Style := bsSolid
        else
          FDrawBitmap.Canvas.Brush.Style := bsClear;
        if cbDrawVoronoiBorder.Checked then
        begin
          FDrawBitmap.Canvas.Pen.Style := psSolid;
          FDrawBitmap.Canvas.Pen.Color := clbVoronoiBorderColor.ButtonColor;
        end else
          FDrawBitmap.Canvas.Pen.Style := psClear;
        for i := 0 to High(FSeedPoints) do
        begin
          SetLength(Pts, Length(FTriangulation.VoronoiPolygon[i]));
          for j := 0 to High(Pts) do
            Pts[j] := FTriangulation.VoronoiPolygon[i][j].Round;
          if cbFillWithColorOfSeedPoints.Checked then
          begin
            x := EnsureRange(round(FSeedPoints[i].X), 0, FImg.Width - 1);
            y := EnsureRange(round(FSeedPoints[i].Y), 0, FImg.Height - 1);
            c := FImg.Colors[x, y];
            FDrawBitmap.Canvas.Brush.Color := FPColorToTColor(c);
          end;
          FDrawBitmap.Canvas.Polygon(Pts);
          Pts := nil;
        end;
      end;
    end;

    // *** Draw seed points ***
    if cbDrawSeedPoints.Checked then
    begin
      if not cbColorFromImage.Checked then
        clr := clBlack;
      if rbConstantSeedPointSize.Checked then
        for i := 0 to High(FSeedPoints) do
        begin
          if cbColorFromImage.Checked then
          begin
            x := EnsureRange(round(FSeedPoints[i].X), 0, FImg.Width - 1);
            y := EnsureRange(round(FSeedPoints[i].Y), 0, FImg.Height - 1);
            c := FImg.Colors[x, y];
            clr := FPColorToTColor(c);
          end;
          FDrawBitmap.Canvas.Pixels[round(FSeedPoints[i].X), round(FSeedPoints[i].Y)] := clr
        end
      else
      begin
        FDrawBitmap.Canvas.Brush.Style := bsSolid;
        FDrawBitmap.Canvas.Pen.Style := psClear;
        for i := 0 to High(FSeedPoints) do
        begin
          P := FSeedPoints[i].Round;
          P.X := EnsureRange(P.X, 0, FDrawBitmap.Width-1);
          P.Y := EnsureRange(P.Y, 0, FDrawBitmap.Height-1);
          c := FImg.Colors[P.X, P.Y];
          if cbColorFromImage.Checked then
            clr := FPColorToTColor(c);
          value := (0.3*c.Red + 0.59*c.Green + 0.11*c.Blue) / 65535;
          r := round(seMaxSeedPtSize.Value * (1.0 - value));
          if r < 1 then
            continue;
          if r < 2 then
            FDrawBitmap.Canvas.Pixels[P.X, P.Y] := clr
          else
          begin
            P.X := P.X - r div 2;
            P.Y := P.Y - r div 2;
            FDrawBitmap.Canvas.Brush.Color := clr;
            FDrawBitmap.Canvas.Ellipse(P.X,P.Y, P.X+r, P.Y+r);
          end;
        end;
      end;
    end;

    ImageBox.Invalidate;

  finally
    Screen.Cursor := crDefault;
  end;
end;

procedure TMainForm.LoadFile(AFileName: String);
begin
  if not FileExists(AFileName) then
  begin
    MessageDlg(Format('File "%s" not found.', [AFileName]), mtError, [mbOk], 0);
    exit;
  end;

  Screen.Cursor := crHourglass;
  try
    FPicture.Free;
    FPicture := TPicture.Create;
    FPicture.LoadFromFile(AFileName);

    FDrawBitmap.Free;
    FDrawBitmap := TBitmap.Create;
    FDrawBitmap.Assign(FPicture.Bitmap);

    FImg.Free;
    FImg := FDrawBitmap.CreateIntfImage;

    FSeedPoints := nil;
    FLloydsAlgorithmNeeded := false;

    InitSeedPoints;
    UpdateImage;
    UpdateFileHistory(AFileName);
  finally
    Screen.Cursor := crDefault;
  end;
end;

procedure TMainForm.LloydsAlgorithm;
const
  FACTOR = 0.95;   // < 1 to avoid creating duplicate points
var
  i, j, n: Integer;
  c: TColor;
  w: array of Double = nil;
  wsum: Double;
  P: TPoint;
  pts: array of TDoublePoint = nil;
begin
  if FTriangulation = nil then
    exit;

  SetLength(pts, Length(FSeedPoints));
  for i := 4 to High(FSeedPoints) do   // i=4:  Skip the four corner points, they do not change
  begin
    n := Length(FTriangulation.VoronoiPolygon[i]);
    SetLength(w, n);
    wsum := 0.0;
    for j := 0 to n-1 do
    begin
      P := FTriangulation.VoronoiPolygon[i][j].Round;
      P.X := EnsureRange(P.X, 0, FImg.Width-1);
      P.Y := EnsureRange(P.Y, 0, FImg.Height-1);
      c := FPColorToTColor(FImg.Colors[P.X, P.Y]);
      w[j] := 1.0 - ColorToGray(c)/255 * FACTOR;
      wsum := wsum + w[j];
    end;
    if wsum <> 0 then
      for j := 0 to n-1 do
        w[j] := w[j] / wsum
    else
      w[j] := 1;
    pts[i].X := 0;
    pts[i].Y := 0;
    for j := 0 to n-1 do
    begin
      pts[i].X := EnsureRange(pts[i].X + FTriangulation.VoronoiPolygon[i][j].X * w[j], 0, FImg.Width-1);
      pts[i].Y := EnsureRange(pts[i].Y + FTriangulation.VoronoiPolygon[i][j].Y * w[j], 0, FImg.Height-1);
    end;
  end;
  for i := 4 to High(FSeedPoints) do
    FSeedPoints[i] := pts[i];
end;

procedure TMainForm.seNumSeedPointsChange(Sender: TObject);
begin
  InitSeedPoints;
  UpdateImage;
end;

procedure TMainForm.UpdateFileHistory(AFileName: String);
begin
  AFileName := ExpandFileName(AFileName);
  if (cbFileName.Items.IndexOf(AFileName) = -1) then
  begin
    cbFileName.Items.Insert(0, AFileName);
    while cbFileName.Items.Count > MAX_HISTORY_COUNT do
      cbFileName.Items.Delete(cbFileName.Items.Count-1);
  end;
end;

procedure TMainForm.ReadIni;
var
  ini: TCustomIniFile;
  i, L, T, W, H: Integer;
  R: TRect;
  List: TStrings;
  s: String;
begin
  ini := CreateIni;
  try
    T := ini.ReadInteger('MainForm', 'Top', Top);
    L := ini.ReadInteger('MainForm', 'Left', Left);
    W := ini.ReadInteger('MainForm', 'Width', Width);
    H := ini.ReadInteger('MainForm', 'Height', Height);

    R := Screen.WorkAreaRect;
    if W > R.Width then W := R.Width;
    if H > R.Height then H := R.Height;
    if L + W > R.Right then L := R.Right - W;
    if T + H > R.Bottom then T := R.Bottom - H;
    SetBounds(L, T, W, H);

    List := TStringList.Create;
    try
      ini.ReadSection('History', List);
      for i := 0 to List.Count-1 do
      begin
        s := ini.ReadString('History', List[i], '');
        if FileExists(s) then
          cbFilename.Items.Add(s);
      end;
      if cbFileName.Items.Count > 0 then
      begin
        cbFileName.Text := cbFileName.Items[0];
        LoadFile(cbFileName.Text);
      end;
    finally
      List.Free;
    end;
  finally
    ini.Free;
  end;
end;

procedure TMainForm.WriteIni;
var
  ini: TCustomIniFile;
  i: Integer;
begin
  ini := CreateIni;
  try
    ini.WriteInteger('MainForm', 'Top', Top);
    ini.WriteInteger('MainForm', 'Left', Left);
    ini.WriteInteger('MainForm', 'Width', Width);
    ini.WriteInteger('MainForm', 'Height', Height);

    for i := 0 to cbFilename.Items.Count-1 do
      ini.WriteString('History', 'Item'+ IntToStr(i+1), cbFileName.Items[i]);
  finally
    ini.Free;
  end;
end;

end.

