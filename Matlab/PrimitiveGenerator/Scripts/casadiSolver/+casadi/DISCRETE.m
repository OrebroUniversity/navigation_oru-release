function v = DISCRETE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 116);
  end
  v = vInitialized;
end
