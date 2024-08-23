
function varargout = notasMusicales(varargin)
% NOTASMUSICALES MATLAB code for notasMusicales.fig
%      NOTASMUSICALES, by itself, creates a new NOTASMUSICALES or raises the existing
%      singleton*.
%
%      H = NOTASMUSICALES returns the handle to a new NOTASMUSICALES or the handle to
%      the existing singleton*.
%
%      NOTASMUSICALES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in NOTASMUSICALES.M with the given input arguments.
%
%      NOTASMUSICALES('Property','Value',...) creates a new NOTASMUSICALES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before notasMusicales_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to notasMusicales_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help notasMusicales

% Last Modified by GUIDE v2.5 17-Jun-2024 21:31:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @notasMusicales_OpeningFcn, ...
                   'gui_OutputFcn',  @notasMusicales_OutputFcn, ...
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


% --- Executes just before notasMusicales is made visible.
function notasMusicales_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to notasMusicales (see VARARGIN)

% Choose default command line output for notasMusicales
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes notasMusicales wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = notasMusicales_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;













% Variables globales
global distance5;
global distance1;
global distanceZ;
distance5 = zeros(1, 7); % Inicializa con ceros
distance1 = zeros(1, 7); 
distanceZ = zeros(1, 7);

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%
%
%            ¡¡¡¡¡¡¡¡¡¡¡¡  CAMBIAR LA RUTA QUE CONTIENE A LA CARPETA "Notas Musicales"   !!!!!!!!!!!
%
%                               
%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
global ruta_base;
ruta_base = 'E:\Lenovo\Documents\DIAMCRUST\SCHOOL\UAEM ICO\MATERIAS\SEMESTRE 2024-A\PROC DE IMAG DIGITALES\PROJECT'; 
%           |                                                                                                       |   
%           |____________________________________Cambie Aquí________________________________________________________|





% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
global distance5;
global distance1;
distance5 = [0, 0, 0, 0, 0, 0, 0];
distance1 = [0, 0, 0, 0, 0, 0, 0];
global ruta_base;
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%PARTE 1: ENTRENAMIENTO
imageDir = fullfile(ruta_base, 'Notas musicales', 'TRAINING');
list = dir(fullfile(imageDir, '*.jpg'));
number_of_files = numel(list);


for i = 1:number_of_files(1,1)
    fileName = list(i).name;
    filename = fullfile(imageDir, list(i).name);
    A = imread(filename);
    axes(handles.axes1); imshow(A); title("Imagen Nota de Entrenamiento", 'Color', 'white');
    A= A(:,130:end,:);
    Igray = rgb2gray(A);
    Ibw = imbinarize(Igray, 'adaptive', 'ForegroundPolarity', 'bright', 'Sensitivity', 0.45);

    % Aplicar el filtro de Sobel horizontal para obtener el gradiente en x
    Gx = fspecial('sobel');
    IgradX = imfilter(Igray, Gx, 'replicate');
    
    % Binarizar la imagen del gradiente
    lineasBW = imbinarize(IgradX);
    
    % Limpiar la imagen binaria (opcional)
    lineasBW = bwareaopen(lineasBW, 50); % Eliminar objetos pequeños
    lineasBW = imclose(lineasBW, strel('line', 100, 0)); % Cerrar pequeñas brechas en líneas horizontales
    lineasBW = bwmorph(lineasBW, "fill");
    %figure;imshow(lineasBW);title('Líneas');

    [lines,n] = bwlabel(lineasBW, 4);
    coloredLabels = label2rgb(uint8(lines), 'hsv', 'k', 'shuffle');
    props = regionprops(lines, 'BoundingBox');
    
    % Crear una matriz para almacenar las líneas filtradas
    linea = zeros(size(lineasBW));
    
    for i = 1 : numel(props)
        % Obtener las dimensiones de la bounding box
        bb_width = props(i).BoundingBox(3);
        bb_height = props(i).BoundingBox(4);
        
        % Calcular la relación de aspecto (ancho / alto)
        aspect_ratio = bb_width / bb_height;
        
        % Establecer un umbral para la relación de aspecto (AJUSTABLE)
        aspect_ratio_threshold = 10;  
        
        % Filtrar regiones que tienen una relación de aspecto mayor al umbral
        if aspect_ratio > aspect_ratio_threshold
            % Si es una línea horizontal almacenarla en 'linea'
            bb_x = round(props(i).BoundingBox(1));
            bb_y = round(props(i).BoundingBox(2));
            bb_w = round(props(i).BoundingBox(3));
            bb_h = round(props(i).BoundingBox(4));
            
            % Copiar la región de 'lineasBW' a 'linea'
            linea(bb_y : bb_y + bb_h - 1, bb_x : bb_x + bb_w - 1) = lineasBW(bb_y : bb_y + bb_h - 1, bb_x : bb_x + bb_w - 1);
        end
    end

    % Etiquetar las líneas filtradas
    [lines_filt,n_filt] = bwlabel(linea, 4);
    coloredLabels_filt = label2rgb(uint8(lines_filt), 'hsv', 'k', 'shuffle');
    
    % Mostrar las líneas etiquetadas filtradas
    %figure; imshow(coloredLabels_filt); title('Líneas horizontales etiquetadas');
    axes(handles.axes2); imshow(coloredLabels_filt); title('Pentagrama detectado', 'Color', 'white');
    % Seleccionar la región etiquetada número 4
    region_num = 4;
    mask = ismember(lines_filt, region_num);
    
    % Mostrar la región etiquetada número 4
    %figure; imshow(mask); title(['Región etiquetada número ', num2str(region_num)]);
    
    num_region = 5;
    mask2 = ismember(lines_filt, num_region);
    
    % Mostrar la región etiquetada número 4
    %figure; imshow(mask); title(['Región etiquetada número ', num2str(num_region)]);

    propsLF = regionprops(lines_filt, 'Centroid');
    disp(n_filt);  % Mostrar el número de líneas horizontales encontradas

    %DETECCION DE CIRCULOS
    Ibw = ~Ibw;

    % Estructurante para la morfología y quitar lineas del pentagrama
    se = strel('line', 30, 0);
    Iopen = imopen(Ibw, se);
    Iclean = (Ibw - Iopen);
    %figure;imshow(~Iclean);title('Líneas Eliminadas e identificacion de circulos');
    

    %estableciendo radios min y max
    Rmin =10; %30
    Rmax = 100; %65
    
    % encuentra los centros de las circunferencias imaginarias claros
    % encuentra el valor de los radios de los circulos oscuros
    [centers, radii] = imfindcircles(~Iclean,[Rmin Rmax],'ObjectPolarity','dark');
    
    a = centers(1,2)
    

    if (fileName == "DO.jpg")
        distance5(1) = centers(1,2)-propsLF(5).Centroid(2)
        distance1(1) = centers(1,2)-propsLF(1).Centroid(2)
        noteName = "DO";
    elseif (fileName == "RE.jpg")
        distance5(2) = centers(1,2)-propsLF(5).Centroid(2)
        distance1(2) = centers(1,2)-propsLF(1).Centroid(2)
        noteName = "RE";
        
    elseif (fileName == "MI.jpg")
        distance5(3) = centers(1,2)-propsLF(5).Centroid(2)
        distance1(3) = centers(1,2)-propsLF(1).Centroid(2)
        noteName = "MI";
        
    elseif (fileName == "FA.jpg")
        distance5(4) = centers(1,2)-propsLF(5).Centroid(2)
        distance1(4) = centers(1,2)-propsLF(1).Centroid(2)
        noteName = "FA";
        
    elseif (fileName == "SOL.jpg")
        distance5(5) = centers(1,2)-propsLF(5).Centroid(2)
        distance1(5) = centers(1,2)-propsLF(1).Centroid(2)
        noteName = "SOL";
        
    elseif (fileName == "LA.jpg")
        distance5(6) = centers(1,2)-propsLF(5).Centroid(2)
        distance1(6) = centers(1,2)-propsLF(1).Centroid(2)
        noteName = "LA";
        
    elseif (fileName == "SI.jpg")
        distance5(7) = centers(1,2)-propsLF(5).Centroid(2)
        distance1(7) = centers(1,2)-propsLF(1).Centroid(2)
        noteName = "SI";
        
    end
    axes(handles.axes3); imshow(Igray); title('Clasificación de la Nota: '+noteName, 'Color', 'white');
    viscircles(centers, radii,'EdgeColor','b');
    pause(1);
    %close all;
end

fprintf("Se determina como buen descriptor estable la distancia1 que refiere a la distancia entre \n la nota y la primera linea (hasta arriba) del pentagrama musical");
%-------------------DECISIÓN DE DESCRIPTOR---------------------------------
% Se determina como buen descriptor estable la distancia1 que refiere a la distancia 
% entre la nota y la primera linea hasta arriba del pentagrama musical




% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global distance5;
global distance1;
global ruta_base;
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%PARTE 2: PRUEBA
imageDir = fullfile(ruta_base, 'Notas musicales', 'TESTING');
listP = dir(fullfile(imageDir, '*.jpg'));
number_of_files = numel(listP);


for i = 1:number_of_files(1,1)
    filename = fullfile(imageDir, listP(i).name);
    P = imread(filename);
    axes(handles.axes4); imshow(P); title('Partitura', 'Color', 'white');
    Prec= P(:,130:end,:);
    IgrayP = rgb2gray(Prec);
    Pbw = imbinarize(IgrayP, 'adaptive', 'ForegroundPolarity', 'bright', 'Sensitivity', 0.45);
    Pbw2 = imbinarize(IgrayP);

    % Aplicar el filtro de Sobel horizontal para obtener el gradiente en x
    GxP = fspecial('sobel');
    IgradXP = imfilter(IgrayP, GxP, 'replicate');
    
    % Binarizar la imagen del gradiente
    lineasBWP = imbinarize(IgradXP);
    
    % Limpiar la imagen binaria (opcional)
    lineasBWP = bwareaopen(lineasBWP, 50); % Eliminar objetos pequeños
    lineasBWP = imclose(lineasBWP, strel('line', 100, 0)); % Cerrar pequeñas brechas en líneas horizontales
    lineasBWP = bwmorph(lineasBWP, "fill");
    %figure;imshow(lineasBWP);title('Líneas');

    [linesP,nP] = bwlabel(lineasBWP, 4);
    coloredLabelsP = label2rgb(uint8(linesP), 'hsv', 'k', 'shuffle');
    propsP = regionprops(linesP, 'BoundingBox');
    
    % Crear una matriz para almacenar las líneas filtradas
    lineaP = zeros(size(lineasBWP));
    
    for j = 1 : numel(propsP)
        % Obtener las dimensiones de la bounding box
        bb_width = propsP(j).BoundingBox(3);
        bb_height = propsP(j).BoundingBox(4);
        
        % Calcular la relación de aspecto (ancho / alto)
        aspect_ratio = bb_width / bb_height;
        
        % Establecer un umbral para la relación de aspecto (AJUSTABLE)
        aspect_ratio_threshold = 10;  
        
        % Filtrar regiones que tienen una relación de aspecto mayor al umbral
        if aspect_ratio > aspect_ratio_threshold
            % Si es una línea horizontal almacenarla en 'linea'
            bb_x = round(propsP(j).BoundingBox(1));
            bb_y = round(propsP(j).BoundingBox(2));
            bb_w = round(propsP(j).BoundingBox(3));
            bb_h = round(propsP(j).BoundingBox(4));
            
            % Copiar la región de 'lineasBW' a 'linea'
            lineaP(bb_y : bb_y + bb_h - 1, bb_x : bb_x + bb_w - 1) = lineasBWP(bb_y : bb_y + bb_h - 1, bb_x : bb_x + bb_w - 1);
        end
    end

    % Etiquetar las líneas filtradas
    [lines_filtP,n_filtP] = bwlabel(lineaP, 4);
    coloredLabels_filtP = label2rgb(uint8(lines_filtP), 'hsv', 'k', 'shuffle');
    propsLFP = regionprops(lines_filtP, 'Centroid');

    %DETECCION DE CIRCULOS
    Pbw = ~Pbw;

    % Estructurante para la morfología y quitar lineas del pentagrama
    se = strel('line', 35, 0);
    IopenP = imopen(Pbw, se);
    IcleanP = (Pbw2 - ~lineasBWP);

    %estableciendo radios min y max
    RminP =10; %30
    RmaxP = 50; %65
    
    % encuentra el valor de los radios de los circulos oscuros
    [centersP, radiiP] = imfindcircles(~IcleanP,[RminP RmaxP],'ObjectPolarity','dark');
    
   
    numCircles = numel(centersP)/2;
    if numCircles>0
        % Ordenar los centros de los círculos por su coordenada x
        [~, sortedIdx] = sort(centersP(:, 1));
        sortedCentersP = centersP(sortedIdx, :);
        sortedRadiiP = radiiP(sortedIdx);

        for num = 1 :numCircles
            a = sortedCentersP(num,2);
            b = propsLFP(1).Centroid(2);
            distanceP = sortedCentersP(num,2) - propsLFP(1).Centroid(2);
            umbral_Error = 7; %7 pixeles de error
    
            centro_x = sortedCentersP(num, 1);  % Coordenada x del centro
            centro_y = sortedCentersP(num, 2);  % Coordenada y del centro
            
            if distanceP>=distance1(1)-umbral_Error
                nameNota="DO";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','DO.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
    
            elseif distanceP>=distance1(2)-umbral_Error && distanceP<distance1(2)+umbral_Error
                nameNota="RE";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','RE.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
            
            elseif distanceP>=distance1(3)-umbral_Error
                nameNota="MI";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','MI.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
                
            elseif distanceP>=distance1(4) 
                nameNota="FA";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','FA.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
    
            elseif distanceP>=distance1(5)-umbral_Error
                nameNota="SOL";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','SOL.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
               
            elseif distanceP>=distance1(6)
                nameNota="LA";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','LA.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
                
            elseif distanceP>=distance1(7)
                nameNota="SI";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','SI.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
            else
                nameNota="DESCONOCIDO";
            end
           
            axes(handles.axes5); imshow(Pbw2); title('Identificación de la Nota: '+nameNota, 'Color', 'white');
            viscircles([centro_x, centro_y], sortedRadiiP(num), 'EdgeColor', 'r'); 
            pause(1);
            
        end
    elseif numCircles == 0
                nameNota="Silencio";
                ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','SILENCE.wav');
                [y, Fs] = audioread(ruta_audios);
                sound(y,Fs);
                axes(handles.axes5); imshow(Pbw2); title('Identificación de la Nota: '+nameNota, 'Color', 'white');
    end
    pause(2);
end


global distanceZ;
distanceZ = [101, 89, 79, 71, 60, 50, 40];
imageDir = fullfile(ruta_base,"Notas musicales",'TESTING','IMG8.png');
S = imread(imageDir);
axes(handles.axes4); imshow(S); title('Partitura', 'Color', 'white');
Srec= S(:,100:end,:);
IgrayS = rgb2gray(Srec);
Sbw = imbinarize(IgrayS, 'adaptive', 'ForegroundPolarity', 'bright', 'Sensitivity', 0.45);
Sbw2 = imbinarize(IgrayS);

% Aplicar el filtro de Sobel horizontal para obtener el gradiente en x
GxS = fspecial('sobel');
IgradXS = imfilter(IgrayS, GxS, 'replicate');

% Binarizar la imagen del gradiente
lineasBWS = imbinarize(IgradXS);

% Limpiar la imagen binaria (opcional)
lineasBWS = bwareaopen(lineasBWS, 50); % Eliminar objetos pequeños
lineasBWS = imclose(lineasBWS, strel('line', 100, 0)); % Cerrar pequeñas brechas en líneas horizontales
lineasBWS = bwmorph(lineasBWS, "fill");

[linesS,nS] = bwlabel(lineasBWS, 4);
coloredLabelsS = label2rgb(uint8(linesS), 'hsv', 'k', 'shuffle');
propsS = regionprops(linesS, 'BoundingBox');

% Crear una matriz para almacenar las líneas filtradas
lineaS = zeros(size(lineasBWS));

for j = 1 : numel(propsS)
    % Obtener las dimensiones de la bounding box
    bb_width = propsS(j).BoundingBox(3);
    bb_height = propsS(j).BoundingBox(4);
    
    % Calcular la relación de aspecto (ancho / alto)
    aspect_ratio = bb_width / bb_height;
    
    % Establecer un umbral para la relación de aspecto (AJUSTABLE)
    aspect_ratio_threshold = 10;  
    
    % Filtrar regiones que tienen una relación de aspecto mayor al umbral
    if aspect_ratio > aspect_ratio_threshold
        % Si es una línea horizontal almacenarla en 'linea'
        bb_x = round(propsS(j).BoundingBox(1));
        bb_y = round(propsS(j).BoundingBox(2));
        bb_w = round(propsS(j).BoundingBox(3));
        bb_h = round(propsS(j).BoundingBox(4));
        
        % Copiar la región de 'lineasBW' a 'linea'
        lineaS(bb_y : bb_y + bb_h - 1, bb_x : bb_x + bb_w - 1) = lineasBWS(bb_y : bb_y + bb_h - 1, bb_x : bb_x + bb_w - 1);
    end
end

% Etiquetar las líneas filtradas
[lines_filtS,n_filtS] = bwlabel(lineaS, 4);
coloredLabels_filtS = label2rgb(uint8(lines_filtS), 'hsv', 'k', 'shuffle');
propsLFS = regionprops(lines_filtS, 'Centroid');

%DETECCION DE CIRCULOS
Sbw = ~Sbw;

% Estructurante para la morfología y quitar lineas del pentagrama
se = strel('line', 35, 0);
IopenS = imopen(Sbw, se);
IcleanS = (Sbw2 - ~lineasBWS);


%estableciendo radios min y max
RminS =10; %30
RmaxS = 50; %65

% encuentra el valor de los radios de los circulos oscuros
[centersS, radiiS] = imfindcircles(~IcleanS,[RminS RmaxS],'ObjectPolarity','dark');
numCirclesS = numel(centersS)/2;

% Ordenar los centros de los círculos por su coordenada x
[~, sortedIdxS] = sort(centersS(:, 1));
sortedCentersS = centersS(sortedIdxS, :);
sortedRadiiS = radiiS(sortedIdxS);

for num = 1 :numCirclesS
    a = sortedCentersS(num,2);
    b = propsLFS(1).Centroid(2);
    distanceS = sortedCentersS(num,2) - propsLFS(1).Centroid(2);
    umbral_Error = 1; %7 pixeles de error

    centro_xS = sortedCentersS(num, 1);  % Coordenada x del centro
    centro_yS = sortedCentersS(num, 2);  % Coordenada y del centro
    

    if distanceS>=distanceZ(1)
        nameNota="DO";
        ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','DO.wav');
        [y, Fs] = audioread(ruta_audios);
        sound(y,Fs);
    
    elseif distanceS<distanceZ(1) && distanceS>=distanceZ(2)
        nameNota="RE";
        ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','RE.wav');
        [y, Fs] = audioread(ruta_audios);
        sound(y,Fs);
    
    elseif distanceS<distanceZ(2) && distanceS>=distanceZ(3)
        nameNota="MI";
        ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','MI.wav');
        [y, Fs] = audioread(ruta_audios);
        sound(y,Fs);
    
        
    elseif distanceS<distanceZ(3) && distanceS>=distanceZ(4)
        nameNota="FA";
        ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','FA.wav');
        [y, Fs] = audioread(ruta_audios);
        sound(y,Fs);
    
    elseif distanceS<distanceZ(4) && distanceS>=distanceZ(5)
        nameNota="SOL";
        ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','SOL.wav');
        [y, Fs] = audioread(ruta_audios);
        sound(y,Fs);
    
    elseif distanceS<distanceZ(5) && distanceS>=distanceZ(6)
        nameNota="LA";
        ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','LA.wav');
        [y, Fs] = audioread(ruta_audios);
        sound(y,Fs);
    
    elseif distanceS<=distanceZ(7)+umbral_Error
        nameNota="SI";
        ruta_audios = fullfile(ruta_base, 'Notas musicales', 'AUDIOS','SI.wav');
        [y, Fs] = audioread(ruta_audios);
        sound(y,Fs);
    else
        nameNota="DESCONOCIDO";
    end
   
    axes(handles.axes5); imshow(Sbw2); title('Identificación de la Nota: '+nameNota, 'Color', 'white');
    viscircles([centro_xS, centro_yS], sortedRadiiS(num), 'EdgeColor', 'r'); 
    pause(1);
        
    end
