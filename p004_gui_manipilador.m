function varargout = p004_gui_manipilador(varargin)
% P004_GUI_MANIPILADOR MATLAB code for p004_gui_manipilador.fig
%      P004_GUI_MANIPILADOR, by itself, creates a new P004_GUI_MANIPILADOR or raises the existing
%      singleton*.
%
%      H = P004_GUI_MANIPILADOR returns the handle to a new P004_GUI_MANIPILADOR or the handle to
%      the existing singleton*.
%
%      P004_GUI_MANIPILADOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in P004_GUI_MANIPILADOR.M with the given input arguments.
%
%      P004_GUI_MANIPILADOR('Property','Value',...) creates a new P004_GUI_MANIPILADOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before p004_gui_manipilador_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to p004_gui_manipilador_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help p004_gui_manipilador

% Last Modified by GUIDE v2.5 06-Nov-2016 19:37:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @p004_gui_manipilador_OpeningFcn, ...
                   'gui_OutputFcn',  @p004_gui_manipilador_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before p004_gui_manipilador is made visible.
function p004_gui_manipilador_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to p004_gui_manipilador (see VARARGIN)

% Choose default command line output for p004_gui_manipilador
handles.output = hObject;
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%COLOCAR IMAGENES EN LOS BOTONES
[IMAG_UP_L,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\u_l.png');
set(handles.pushbutton_up_l,'CData',IMAG_UP_L);
[IMAG_UP,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\up.png');
set(handles.pushbutton_up,'CData',IMAG_UP);
[IMAG_UP_R,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\u_r.png');
set(handles.pushbutton_up_r,'CData',IMAG_UP_R);
[IMAG_LEFT,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\left.png');
set(handles.pushbutton_left,'CData',IMAG_LEFT);
[IMAG_RIGHT,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\right.png');
set(handles.pushbutton_right,'CData',IMAG_RIGHT);
[IMAG_DOWN_L,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\d_l.png');
set(handles.pushbutton_down_l,'CData',IMAG_DOWN_L);
[IMAG_DOWN_R,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\d_r.png');
set(handles.pushbutton_down_r,'CData',IMAG_DOWN_R);
[IMAG_DOWN,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\down.png');
set(handles.pushbutton_down,'CData',IMAG_DOWN);
[IMAG_HOME,MAP]=imread('\\Vboxsvr\source\Trabajos 2016 - 2\TT\INTERFAZ\Matlab\Pruebas\img_buttons\home.png');
set(handles.pushbutton_home,'CData',IMAG_HOME);

%FUNCION DE SERVICIO DE INTERRUPCION DEL GRAFICO (CLICK DERECHO)
set(hObject,'WindowButtonDownFcn',{@Click_figure_callback,hObject});

%BORRA CONTENIDO EDIT_TEXT
set(handles.edit_lados,'String','');
set(handles.edit_radio,'String','');
set(handles.edit_posX,'String','');
set(handles.edit_posY,'String','');
set(handles.text_state,'String','');

%MENSAJE DE ESTADO DEL SISTEMA
set(handles.text_state,'String','En espera de establecer conexión');

%SELECCION PREDEFINIDA PARA ESCALA DE AVANCE
set(handles.panel_avance,'selectedobject',handles.radiobutton_1cm)

%VARIABLES GLOBALES
%ESTADO DE LA COMUNICACION
IsConnect = 0;  % 1 = Connect,0 = Disconnect
handles.status_com = IsConnect;
global hand LIM_1 LIM_2 LIM_3 LIM_4 HOME_X HOME_Y LADOS_CIRCULO PAUSE STATE CANCEL DELAY_MOV
LIM_1=[10 55];
LIM_2=[10 20];
LIM_3=[110 20];
LIM_4=[110 55];
HOME_X=60;
HOME_Y=55;
LADOS_CIRCULO=40;
PAUSE=0;            %1=PAUSE ENABLE
STATE=1;            %1=AVALILABLE 0=COMMAND SENT
CANCEL=0;           %1=CANCEL ENABLE
DELAY_MOV=0.1;
%COPIA DEL HANDLES PARA LAS FUNCIONES
hand = handles;
%SE DESACTIVAN LOS CONTROLES
disable_controls(handles);
set(handles.pushbutton_connect,'enable','on');
set(handles.pushbutton_cancel,'enable','off');
set(handles.pushbutton_pause,'enable','off');
%set(handles.pushbutton_pause,'enable','off');
%SE DIBUJA EL CONTORNO DEL ESPACIO DE TRABAJO
draw_system(handles);

handles.output = hObject;
%ACTUALIZAR ESTRUCTURA DEL HANDLES
guidata(hObject, handles);

% UIWAIT makes p004_gui_manipilador wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = p004_gui_manipilador_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in radiobutton_polygon.
function radiobutton_polygon_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_polygon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton_polygon


% --- Executes on button press in radiobutton_circle_2.
function radiobutton_circle_2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_circle_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton_circle_2



function edit_radio_Callback(hObject, eventdata, handles)
% hObject    handle to edit_radio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_radio as text
%        str2double(get(hObject,'String')) returns contents of edit_radio as a double
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
input_org = get(hObject, 'String');
input = str2double(input_org);
if isnan(input)||(input<2)||(input>15)
    set(hObject, 'String', '');
    warndlg('El valor del radio debe ser un número en el rango de 2 a 15','Entrada inválida');
end

% --- Executes during object creation, after setting all properties.
function edit_radio_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_radio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_lados_Callback(hObject, eventdata, handles)
% hObject    handle to edit_lados (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_lados as text
%        str2double(get(hObject,'String')) returns contents of edit_lados as a double
%:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
input_org = get(hObject, 'String');
input = str2double(input_org);
if isnan(input)||~isempty(strfind('500','.'))||(input<3)||(input>50)
    set(hObject, 'String', '');
    warndlg('El número de lados debe ser un número entero en el rango de 3 a 50','Entrada inválida');
end
