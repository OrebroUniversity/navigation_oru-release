function v = L_DICT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 5);
  end
  v = vInitialized;
end
