function v = CAT_DERIVATIVE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 127);
  end
  v = vInitialized;
end
