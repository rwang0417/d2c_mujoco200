%% system parameters
modelfile='tower.xml';
mass=strings(1,15);
%% read elements from xml file
xmlmodel=xmlread(modelfile);
geoms=xmlmodel.getElementsByTagName('geom');
%% modify mass
for i=1:1:geoms.getLength-2 % item(0) is in the default section, last item is the floor
element=geoms.item(i); 
element.getAttribute('mass');
element.setAttribute('mass',mass(1,i));
end
xmlwrite(modelfile,xmlmodel)