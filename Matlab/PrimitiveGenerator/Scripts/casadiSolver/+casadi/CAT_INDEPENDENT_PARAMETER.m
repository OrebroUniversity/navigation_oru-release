function v = CAT_INDEPENDENT_PARAMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 132);
  end
  v = vInitialized;
end
