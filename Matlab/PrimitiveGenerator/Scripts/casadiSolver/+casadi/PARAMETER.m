function v = PARAMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 115);
  end
  v = vInitialized;
end
