function v = CAT_INDEPENDENT_CONSTANT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 130);
  end
  v = vInitialized;
end
