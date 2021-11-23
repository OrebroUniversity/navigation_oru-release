function v = OPTI_GENERIC_EQUALITY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 154);
  end
  v = vInitialized;
end
