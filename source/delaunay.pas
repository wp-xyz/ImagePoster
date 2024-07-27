{ Delaunay Tesselation

  Credits to Paul Bourke (pbourke@swin.edu.au) for the original Fortran 77 Program :))
  Conversion to Visual Basic by EluZioN (EluZioN@casesladder.com)
  Conversion from VB to Delphi6 by Dr Steve Evans (steve@lociuk.com)
  Conversion from Delphi6 to FreePascal by Corpsman (corpsman@corpsman.de)

  June 2002 Update by Dr Steve Evans (steve@lociuk.com): Heap memory allocation
  added to prevent stack overflow when MaxVertices and MaxTriangles are very large.

  Additional Updates in June 2002:
  Bug in InCircle function fixed. Radius r := Sqrt(rsqr).
  Check for duplicate points added when inserting new point.
  For speed, all points pre-sorted in x direction using quicksort algorithm and
  triangles flagged when no longer needed. The circumcircle centre and radius of
  the triangles are now stored to improve calculation time.

  October 2012 Update by Corpsman (corpsman@corpsman.de): Added dynamical arrays
  Bug Fixed in calculating the outer triangle position where too small
  Added more comments in the code

  April 2023 Integration into TAChart by Werner Pamler

  You can use this code however you like providing the above credits remain intact
}

unit Delaunay;

{$mode ObjFPC}{$H+}
{$modeswitch advancedrecords}
{$WARN 6058 off : Call to subroutine "$1" marked as inline is not inlined}

interface

uses
  Classes, SysUtils, Types, fgl;

type
  TDoublePoint = record
    X, Y: Double;
    function Round: TPoint;
  end;
  PDoublePoint = ^TDoublePoint;
  TDoublePointArray = array of TDoublePoint;

  TDelaunayTriangle = record
    VertexIndex: array[0..2] of Integer;  // Vertex index in vertex list
    Neighbors: array[0..2] of Integer;    // Index of adjacent triangles in triangle list
    VoronoiPoint: TDoublePoint;
    VoronoiRadius: Double;
  end;
  TDelaunayTriangleArray = array of TDelaunayTriangle;

  TTriangulation = class
  private
    type
      TInternalVertex = record
        P: TDoublePoint;
        SavedIndex: Integer;
        Triangles: TIntegerDynArray;
        class operator = (A, B: TInternalVertex): Boolean;
      end;
      TInternalVertexList = specialize TFPGList<TInternalVertex>;
      TInternalTriangle = record
        VertexIndex: array[0..2] of Integer;
        PreCalc: Boolean;
        Complete: Boolean;   // If True, then all calculations of this triangle are finished (triangle will never be changed again)
        CircumCircleCenter: TDoublePoint;
        CircumCircleRadius: Double;
      end;
      TInternalEdge = record
        StartIndex, EndIndex: Integer;
      end;
  private
    FErrorMsg: String;
    FInternalVertexList: TInternalVertexList;
    FInternalTriangles: array of TInternalTriangle;
    FResultsAvail: Boolean;
    FPoints: PDoublePoint;
    FNumPoints: Integer;
    FTriangles: TDelaunayTriangleArray;
    function GetVertex(AVertexIndex: Integer): TDoublePoint; inline;
    function GetVoronoiPolygon(AVertexIndex: Integer): TDoublePointArray;
  protected
    procedure CalcVoronoiPoints;
    function CircumCircle(A, B, C: TDoublePoint;
      out ACenter: TDoublePoint; out ARadius: Double): Boolean;
    function FindTrianglesAroundVertex(AVertexIndex: Integer): TIntegerDynArray;
    procedure FindTrianglesAroundVertices;
    procedure FixInternalVertexList;
    function InitEdge(AStartIndex, AEndIndex: Integer): TInternalEdge;
    function PointInCircle(const P: TDoublePoint; var ATriangle: TInternalTriangle;
      out C: TDoublePoint; out R: Double): Boolean;
    function PreparePoints(out AMinX, AMinY, AMaxX, AMaxY: Double): Boolean;
    function SameEdges(const AEdge1, AEdge2: TInternalEdge): Boolean;
  public
    constructor Create(AFirstPoint: PDoublePoint; ANumPoints: Integer);
    destructor Destroy; override;
    function Execute: Boolean;
    property ErrorMsg: String read FErrorMsg;
    property Triangles: TDelaunayTriangleArray read FTriangles;
    property Vertex[AVertexIndex: Integer]: TDoublePoint read GetVertex;
    property VoronoiPolygon[AVertexIndex: Integer]: TDoublePointArray read GetVoronoiPolygon;
  end;

function DoublePoint(X, Y: Double): TDoublePoint;


implementation

uses
  Math, IntegerList;

const
  EPS = 1E-9;

function CompareByX(const Vertex1, Vertex2: TTriangulation.TInternalVertex): Integer;
begin
  Result := Sign(Vertex1.P.X - Vertex2.P.X);
end;

function CompareBySavedIndex(const Vertex1, Vertex2: TTriangulation.TInternalVertex): Integer;
begin
  Result := Vertex1.SavedIndex - Vertex2.SavedIndex;
end;

function TDoublePoint.Round: TPoint;
begin
  Result.X := System.Round(X);
  Result.Y := System.Round(Y);
end;

function DoublePoint(X, Y: Double): TDoublePoint;
begin
  Result.X := X;
  Result.Y := Y;
end;

function PointDist(A, B: TDoublePoint): Double;
begin
  Result := sqrt(sqr(A.X - B.X) + sqr(A.Y - B.Y));
end;

procedure UpdateMinMax(AValue: Double; var AMin, AMax: Double);
begin
  if AValue < AMin then AMin := AValue;
  if AValue > AMax then AMax := AValue;
end;

operator +(A, B: TDoublePoint): TDoublePoint;
begin
  Result := DoublePoint(A.X + B.X, A.Y + B.Y);
end;

operator -(A, B: TDoublePoint): TDoublePoint;
begin
  Result := DoublePoint(A.X - B.X, A.Y - B.Y);
end;

operator *(A: TDoublePoint; B: Double): TDoublePoint;
begin
  Result := DoublePoint(A.X * B, A.Y * B);
end;


{ TTrianglulation.TInternalVertex }

class operator TTriangulation.TInternalVertex.=(A, B: TTriangulation.TInternalVertex): Boolean;
begin
  Result := SameValue(A.P.X, B.P.X, EPS) and SameValue(A.P.Y, B.P.Y, EPS);
end;


{ TTriangulation }

constructor TTriangulation.Create(AFirstPoint: PDoublePoint; ANumPoints: Integer);
begin
  inherited Create;
  FPoints := AFirstPoint;
  FNumPoints := ANumPoints;
  FInternalVertexList := TInternalVertexList.Create;
end;

destructor TTriangulation.Destroy;
begin
  FInternalVertexList.Free;
  inherited;
end;

procedure TTriangulation.CalcVoronoiPoints;
var
  i: Integer;
  P1, P2, P3: TDoublePoint;
  C: TDoublePoint;
  R: Double;
begin
  for i := 0 to High(FTriangles) do
  begin
    P1 := FInternalVertexList[FTriangles[i].VertexIndex[0]].P;
    P2 := FInternalVertexList[FTriangles[i].VertexIndex[1]].P;
    P3 := FInternalVertexList[FTriangles[i].VertexIndex[2]].P;
    CircumCircle(P1, P2, P3, C, R);
    FTriangles[i].VoronoiPoint := C;
    FTriangles[i].VoronoiRadius := R;
  end;
end;

{ Calculates the circumcircle of the three specified points, A, B, C, and
  returns its center point and radius.
  If the points are collinear the function returns false as well as a
  negative radius (or zero).
  see: https://en.wikipedia.org/wiki/Circumscribed_circle }
function TTriangulation.CircumCircle(A, B, C: TDoublePoint;
  out ACenter: TDoublePoint; out ARadius: Double): Boolean;
const
  EPS = 1E-8;
var
  ab: TDoublePoint;   // Vector difference A - B
  cb: TDoublePoint;   // Vector difference C - B
  ab2: Double;        // Square of length A - B
  cb2: Double;        // Square of length C - B
  r2: Double;
  ctr: TDoublePoint;
  det: Double;
begin
  ab := A - B;
  cb := C - B;
  det := ab.x * cb.y - cb.x * ab.y;

  { When the determinant is 0 all three points are collinear, and the
    circumcircle does not exist. In this case the special situation is marked
    by a negative radius, and the function returns false. }
  if SameValue(det, 0.0, EPS) then
  begin
    ACenter := (A + B + C) * (1/3);
    ARadius := -PointDist(B, ACenter);
    Result := false;
    exit;
  end;

  ab2 := sqr(ab.x) + sqr(ab.y);
  cb2 := sqr(cb.x) + sqr(cb.y);
  ctr := DoublePoint(ab2 * cb.y - cb2 * ab.y, cb2 * ab.x - ab2 * cb.x) * (0.5/det);
  r2 := sqr(ctr.x) + sqr(ctr.y);
  ACenter := ctr + B;
  ARadius := sqrt(r2);
  Result := true;
end;

{ Runs the triangulation }
function TTriangulation.Execute: Boolean;
const
  BLOCK_SIZE = 1000; // Allocating Memory in Blocks, keep allocating overhead small, and gives dynamic allocation
var
  P1, P2, P3: TDoublePoint;
  C: TDoublePoint;
  R: Double;
  minx, miny, maxx, maxy: Double;
  dmax, xmid, ymid: Double;
  nVert: Integer = 0;   // Counter for all Vertices
  nTri: Integer = 0;    // Counter for triangles
  nEdges: Integer = 0;  // Counter for edges
  v: TInternalVertex = (P:(X:0.0; Y:0); SavedIndex:-1; Triangles: nil;);
  edges: array of TInternalEdge = nil;
  i, j, k: Integer;
  inCircle: Boolean;
begin
  FResultsAvail := false;
  FErrorMsg := '';
  FTriangles := nil;
  FInternalVertexList.Clear;

  // Not enough points
  if FNumPoints < 3 then begin
    Result := false;
    FErrorMsg := 'Not enough points for Delaunay tesselation.';
    exit;
  end;

  // Trivial solution of 3 points
  if FNumPoints = 3 then begin
    SetLength(FTriangles, 1);
    FTriangles[0].VertexIndex[0] := 0;
    FTriangles[0].VertexIndex[1] := 1;
    FTriangles[0].VertexIndex[2] := 2;
    CircumCircle(FPoints^, (FPoints+1)^, (FPoints+2)^, FTriangles[0].VoronoiPoint, r);
    FResultsAvail := true;
    exit;
  end;

  { Copy points into internal data structures, calculate bound box,
    check for duplicates }
  if not PreparePoints(minx, miny, maxx, maxy) then
  begin
    Result := false;
    exit;
  end;
  nVert := FInternalVertexList.Count;

  { Sort points by x to speed up insertion process }
  FInternalVertexList.Sort(@CompareByX);

  { The double edge detection will use the vertex at index 0 to detect invalid edges
    Therefore we insert a dummy vertex at index 0 into the vertext list. }
  FInternalVertexList.Insert(0, v);

  { The outer-most triangle ("super triangle") has to be far away,
    otherwise there could be some seldom cases in which the convex hull is
    not calculated correctly.
    Unfortunatulely, if you choose the factor 20 too large (e.g. 100) there
    are some other errors (in the circumcircle routine).
    The vertices of the super triangle are added to the end of the vertex list
    (Index nVert, nVert+1, nVert+2).
    The net usable triangles are at indices 1..nVert-1 }
  dmax := Max(maxx - minx, maxy - miny) * 20;
  xmid := (maxx + minx) / 2;
  ymid := (maxy + miny) / 2;
  v.SavedIndex := -1;
  v.P.x := (xmid - 2 * dmax);
  v.P.y := (ymid - dmax);
  FInternalVertexList.Add(v);
  v.SavedIndex := -1;
  v.P.x := xmid;
  v.P.y := (ymid + 2 * dmax);
  FInternalVertexList.Add(v);
  v.SavedIndex := -1;
  v.P.x := (xmid + 2 * dmax);
  v.P.y := (ymid - dmax);
  FInternalVertexList.Add(v);

  // Allocate the first array blocks for triangles and edges
  SetLength(FInternalTriangles, BLOCK_SIZE);
  SetLength(edges, BLOCK_SIZE);

  // Insert the super triangle
  FInternalTriangles[1].VertexIndex[0] := nVert + 1;
  FInternalTriangles[1].VertexIndex[1] := nVert + 2;
  FInternalTriangles[1].VertexIndex[2] := nVert + 3;
  FInternalTriangles[1].PreCalc := false;
  FInternalTriangles[1].Complete := false;
  nTri := 1;

  // Insert all points one by one
  for i := 1 to nVert do begin
    nEdges := 0;
    // Set up the edge buffer.
    // If the point (Vertex(i).x,Vertex(i).y) lies inside the circumcircle then the
    // three edges of that triangle are added to the edge buffer.
    j := 0;
    repeat
      inc(j);
      // only check incomplete triangles
      if not FInternalTriangles[j].Complete then
      begin
        inCircle := PointInCircle(FInternalVertexList[i].P, FInternalTriangles[j], C, R);
        //Include this if points are sorted by X
        if (C.x + R) < FInternalVertexList[i].P.x then begin
          FInternalTriangles[j].Complete := True;
        end else
        begin
          if inCircle then begin // if triangle needs partitioning insert edges
            // Reallocate memory if necessary
            if nEdges + 3 > High(edges) then
              SetLength(edges, Length(edges) + BLOCK_SIZE);
            with FInternalTriangles[j] do
            begin
              edges[nEdges+1] := InitEdge(VertexIndex[0], VertexIndex[1]);
              edges[nEdges+2] := InitEdge(VertexIndex[1], VertexIndex[2]);
              edges[nEdges+3] := InitEdge(VertexIndex[2], VertexIndex[0]);
            end;
            inc(nEdges, 3);
            FInternalTriangles[j] := FInternalTriangles[nTri];
            FInternalTriangles[nTri].PreCalc := false;
            FInternalTriangles[j].Complete := FInternalTriangles[nTri].Complete;
            dec(j);
            dec(nTri);
          end;
        end;
      end;
    until j >= nTri;

    // Tag multiple edges
    // Note: if all triangles are specified anticlockwise then all
    // interior edges are pointing in opposite directions.
    for j := 1 to nEdges - 1 do begin
      if (edges[j].StartIndex <> 0) and (edges[j].EndIndex <> 0) then
      begin
        for k := j + 1 to nEdges do begin
          if (edges[k].StartIndex <> 0) and (edges[k].EndIndex <> 0) and SameEdges(edges[j], edges[k]) then
          begin
            edges[j] := InitEdge(0, 0);
            edges[k] := InitEdge(0, 0);
          end;
        end;
      end;
    end;

    { Form new triangles for the current point skipping over any tagged edges.
      All edges are arranged in clockwise order. }
    for j := 1 to nEdges do begin
      if (edges[j].StartIndex <> 0) and (edges[j].EndIndex <> 0) then
      begin
        inc(nTri);
        // Reallocate memory if necessery
        if nTri > High(FInternalTriangles) then
          SetLength(FInternalTriangles, Length(FInternalTriangles) + BLOCK_SIZE);
        FInternalTriangles[nTri].VertexIndex[0] := edges[j].StartIndex;
        FInternalTriangles[nTri].VertexIndex[1] := edges[j].EndIndex;
        FInternalTriangles[nTri].VertexIndex[2] := i;
        FInternalTriangles[nTri].PreCalc := false;
        FInternalTriangles[nTri].Complete := false;
      end;
    end;
  end;   // for i (insert all points one by one)

  { Remove triangles with supertriangle vertices
    These are triangles which have a vertex number greater than nVert. }
  i := 0;
  repeat
    inc(i);
    if (FInternalTriangles[i].VertexIndex[0] > nVert) or
       (FInternalTriangles[i].VertexIndex[1] > nVert) or
       (FInternalTriangles[i].VertexIndex[2] > nVert) then
    begin
      FInternalTriangles[i] := FInternalTriangles[nTri];
      dec(i);
      dec(nTri);
    end;
  until i >= nTri;

  { Convert all results to output format, using the "unsorted" versions of the points }
  SetLength(FTriangles, nTri);
  j := 0;
  for i := 1 to nTri do begin
    FTriangles[j].VertexIndex[0] := FInternalVertexList[FInternalTriangles[i].VertexIndex[0]].SavedIndex;
    FTriangles[j].VertexIndex[1] := FInternalVertexList[FInternalTriangles[i].VertexIndex[1]].SavedIndex;
    FTriangles[j].VertexIndex[2] := FInternalVertexList[FInternalTriangles[i].VertexIndex[2]].SavedIndex;
    FTriangles[j].Neighbors[0] := -1;
    FTriangles[j].Neighbors[1] := -1;
    FTriangles[j].Neighbors[2] := -1;
    inc(j);
  end;

  { Voronoi polygon generation }
  FixInternalVertexList;
  CalcVoronoiPoints;
  FindTrianglesAroundVertices;

  { Free all temporary variables }
  SetLength(edges, 0);
  SetLength(FInternalTriangles, 0);

  FResultsAvail := true;
end;

function TTriangulation.FindTrianglesAroundVertex(AVertexIndex: Integer): TIntegerDynArray;
var
  tri: TIntegerList = nil;

  function NextVertexIndex(i: Integer; Clockwise: Boolean): Integer;
  begin
    if Clockwise then
    begin
      if i = 2 then Result := 0 else Result := i + 1;
    end else
    begin
      if i = 0 then Result := 2 else Result := i - 1;
    end;
  end;

  { Finds, for the given triangle, the index of the next vertex which is
    in clockwise or counterclockwise orientation to AVertexIdx }
  function FindNextVertex(ATriangleIdx, AVertexIdx: Integer; Clockwise: Boolean): Integer;
  var
    T: TDelaunayTriangle;
    i: Integer;
  begin
    T := FTriangles[ATriangleIdx];
    for i := 0 to 2 do
      if T.VertexIndex[i] = AVertexIdx then
      begin
        Result := T.VertexIndex[NextVertexIndex(i, not Clockwise)];  // "not" is correct here
        exit;
      end;
    Result := -1;
  end;

  { Finds the neighbor triangle of ATriangle which shares the specified vertices.
    The neighbor indices are stored in each triangle record. }
  function FindNeighborTriangleWithVertices(ATriangleIdx, AVertexIdx1, AVertexIdx2: Integer): Integer;
  var
    T: TDelaunayTriangle;
    ti, i, j, kp, km, k: Integer;
  begin
    Result := -1;
    for i := 0 to tri.Count-1 do
    begin
      ti := tri[i];
      if ti <> ATriangleIdx then
      begin
        T := FTriangles[ti];
        for j := 0 to 2 do
        begin
          kp := NextVertexIndex(j, true);
          km := NextVertexIndex(j, false);
          if (T.VertexIndex[j] = AVertexIdx1) and
             ((T.VertexIndex[kp] = AVertexIdx2) or (T.VertexIndex[km] = AVertexIdx2)) then
          begin
            Result := ti;
            for k := 0 to 2 do
              if T.Neighbors[k] = -1 then
              begin
                FTriangles[ti].Neighbors[k] := ATriangleIdx;
                break;
              end;
            T := FTriangles[ATriangleIdx];
            for k := 0 to 2 do
              if T.Neighbors[k] = -1 then
              begin
                FTriangles[ATriangleIdx].Neighbors[k] := ti;
                break;
              end;
            exit;
          end;
        end;
      end;
    end;
    Result := -1;
  end;

var
  i, j: Integer;
  ti, ti0: Integer;  // triangle index
  vi: Integer;       // vertex index
  triCW: TIntegerList = nil;
  triCCW: TIntegerList = nil;
begin
  Result := nil;

  tri := TIntegerList.Create;
  triCW := TIntegerList.Create;
  triCCW := TIntegerList.Create;
  try
    { Run through all vertices and store the index of those triangles
      in the list tri which share one vertex with the specified vertex }
    for ti := 0 to High(FTriangles) do
      for vi := 0 to 2 do
        if FTriangles[ti].VertexIndex[vi] = AVertexIndex then
        begin
          tri.Add(ti);
          break;
        end;
    if tri.Count = 0 then
      exit;

    { Rearrange the vertices in clockwise order.
      At first, search in clockwise direction and store the
      found triangle indices in the list triCW }
    ti := tri[0];
    ti0 := ti;
    repeat
      triCW.Add(ti);
      ti := FindNeighborTriangleWithVertices(ti, AVertexIndex, FindNextVertex(ti, AVertexIndex, true));
    until (ti = -1) or (ti = ti0);

    if ti = -1 then
    begin
      { Then search in counterclockwise direction and store the
        found triangle indices in triCCW. }
      ti := tri[0];
      ti0 := ti;
      ti := FindNeighborTriangleWithVertices(ti, AVertexIndex, FindNextVertex(ti, AVertexIndex, false));
      while (ti <> -1) do
      begin
        triCCW.Add(ti);
        ti := FindNeighborTriangleWithVertices(ti, AVertexIndex, FindNextVertex(ti, AVertexIndex, false));
        if ti = ti0 then
          break;
      end;
    end;

    { Merge the triangle indices in the triCW and triCCW lists }
    SetLength(Result, triCW.Count + triCCW.Count);
    j := 0;
    // Note: the counterclockwise list is in reverse order
    for i := triCCW.Count-1 downto 0 do
    begin
      Result[j] := triCCW[i];
      inc(j);
    end;
    for i := 0 to triCW.Count-1 do
    begin
      Result[j] := triCW[i];
      inc(j);
    end;

  finally
    triCCW.Free;
    triCW.Free;
    tri.Free;
  end;
end;

{ Find the triangles around each vertex. }
procedure TTriangulation.FindTrianglesAroundVertices;
var
  vi: Integer;
  v: TInternalVertex;
begin
  for vi := 0 to FInternalVertexList.Count-1 do
  begin
    v := FInternalVertexList[vi];
    v.Triangles := FindTrianglesAroundVertex(vi);
    FInternalVertexList[vi] := v;
  end;
end;

{ For Voronoi generation we need the vertices in the original order
  and without the temporarily added vertices. }
procedure TTriangulation.FixInternalVertexList;
var
  i: Integer;
begin
  // Sort the internal vertex list in the original order
  FInternalVertexList.Sort(@CompareBySavedIndex);

  // Remove the temporarily added vertices (index 0, and supertriangle) from
  // the internal vertex list
  while FInternalVertexList[0].SavedIndex = -1 do
    FInternalVertexList.Delete(0);
end;

function TTriangulation.GetVertex(AVertexIndex: Integer): TDoublePoint;
begin
  Result := (FPoints + AVertexIndex)^;
end;

function TTriangulation.GetVoronoiPolygon(AVertexIndex: Integer): TDoublePointArray;
var
  v: TInternalVertex;
  t: TDelaunayTriangle;
  i: Integer;
begin
  Result := nil;
  v := FInternalVertexList[AVertexIndex];
  SetLength(Result, Length(v.Triangles));
  for i := 0 to High(Result) do
  begin
    t := FTriangles[v.Triangles[i]];
    Result[i] := t.VoronoiPoint;
  end;
end;

function TTriangulation.InitEdge(AStartIndex, AEndIndex: Integer): TInternalEdge;
begin
  Result.StartIndex := AStartIndex;
  Result.EndIndex := AEndIndex;
end;

{ Returns TRUE if the point P lies inside the circumcircle made up by the
  points of the specified triangle.
  The circumcircle center is returned in C and the radius in R.
  NOTE: A point on the edge is considered to be inside the circumcircle. }
function TTriangulation.PointInCircle(const P: TDoublePoint;
  var ATriangle: TInternalTriangle; out C: TDoublePoint; out R: Double): Boolean;
var
  rsqr: double;
  drsqr: double;
  P1, P2, P3: TDoublePoint;
begin
  // Check whether C and R already have been calculated
  if ATriangle.PreCalc then
  begin
    C := ATriangle.CircumCircleCenter;
    R := ATriangle.CircumCircleRadius;
    rsqr := sqr(r);
    drsqr := sqr(P.x - C.x) + sqr(P.y - C.y);
  end else
  begin
    P1 := FInternalVertexList[ATriangle.VertexIndex[0]].P;
    P2 := FInternalVertexList[ATriangle.VertexIndex[1]].P;
    P3 := FInternalVertexList[ATriangle.VertexIndex[2]].P;
    CircumCircle(P1, P2, P3, C, R);
    R := abs(R);
    ATriangle.PreCalc := true;
    ATriangle.CircumCircleCenter := C;
    ATriangle.CircumCircleRadius := R;
    rsqr := sqr(R);
    drsqr := sqr(P.x - C.x) + sqr(P.y - C.y);
  end;
  Result := drsqr <= rsqr;
end;

{ Copies the data points to an internal list and calculates the bounding box
  of the point coordiantes.
  The routine also checks for duplicate points which are not allowed in the
  point set. If this is the case, the result value is false. }
function TTriangulation.PreparePoints(out AMinX, AMinY, AMaxX, AMaxY: Double): Boolean;
var
  i, j: Integer;
  v: TInternalVertex;
begin
  AMinX := FPoints^.X;
  AMinY := FPoints^.Y;
  AMaxX := FPoints^.X;
  AMaxY := FPoints^.Y;
  for i := 0 to FNumPoints-1 do
  begin
    // Add to internal list
    v.P := (FPoints+i)^;
    v.SavedIndex := i;
    FInternalVertexList.Add(v);
    // Update bounding box
    UpdateMinMax(v.P.X, AMinX, AMaxX);
    UpdateMinMax(v.P.Y, AMinY, AMaxY);
    // Check duplicates
    for j := i + 1 to FNumPoints-1 do begin
      if SameValue((FPoints+i)^.x, (FPoints+j)^.x, EPS) and
         SameValue((FPoints+i)^.y, (FPoints+j)^.y, EPS) then
      begin
        Result := false;
        FErrorMsg := Format('Error in Delaunay tesselation: Coincident points #%d and #%d.', [i, j]);
        exit;
      end;
    end;
  end;
  Result := true;
end;

{ Checks whether the two specified edges are coicident, i.e. belong to adjacent
  triangles.
  NOTE: If the vertices in all triangles are specified in anti-clockwise order
  then the interior edges are pointing in opposite directions. }
function TTriangulation.SameEdges(const AEdge1, AEdge2: TInternalEdge): Boolean;
begin
  Result := (AEdge1.StartIndex = AEdge2.EndIndex) and (AEdge1.EndIndex = AEdge2.StartIndex);
end;

end.

