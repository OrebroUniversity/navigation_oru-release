function v = SWIG_IND1()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 10);
  end
  v = vInitialized;
end
