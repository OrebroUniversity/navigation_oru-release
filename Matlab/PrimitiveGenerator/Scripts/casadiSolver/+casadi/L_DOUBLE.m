function v = L_DOUBLE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 4);
  end
  v = vInitialized;
end
