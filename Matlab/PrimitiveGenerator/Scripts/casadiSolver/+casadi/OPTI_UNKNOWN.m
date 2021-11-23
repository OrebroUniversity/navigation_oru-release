function v = OPTI_UNKNOWN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 160);
  end
  v = vInitialized;
end
