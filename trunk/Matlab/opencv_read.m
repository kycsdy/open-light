function M = opencv_read(filename)

% OPENCV_READ Reads XML-formatted files produced by OpenCV.
%    OPENCV_READ Reads files saved using the "cvSave" function in OpenCV.
%
%    This functions using the XML_IO_TOOLS by Jaroslaw Tuszynski. The
%    necessary XML_READ function is included in the XML_IO_TOOLS 
%    subdirectory. The complete package of XML functions can be obtained
%    from Matlab Central at the following URL.
%
%       Jaroslaw Tuszynski
%       "XML_IO_TOOLS"
%       http://www.mathworks.com/matlabcentral/fileexchange/12907
%       6 November 2006 (updated 15 May 2009)
%
%    Please read the SIGGRAPH 2009 course notes for additional details.
%
%       Douglas Lanman and Gabriel Taubin
%       "Build Your Own 3D Scanner: 3D Photography for Beginners"
%       ACM SIGGRAPH 2009 Course Notes    
%
% Douglas Lanman
% Brown University
% June 2009

% Read a XML file (generated by cvSave) and return MATLAB matrix.
C = struct2cell(xml_read(filename));
if ischar(C{1}.data)
   data = sscanf(C{1}.data,'%f')';
   data = data(:);
else
   data = C{1}.data';
   data = data(:);
end
if strcmp(C{1}.dt,'f')
   M = reshape(data, [C{1}.cols C{1}.rows])';
elseif strcmp(C{1}.dt,'"2f"')
   M = reshape(data, [2*C{1}.cols C{1}.rows])';
end